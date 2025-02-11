const { performance } = require("perf_hooks"); // not needed for node > ??

const { KalmanFilter, pwrStdev } = require("./kalman-filter");

/** @typedef {import("node-red").NodeMessage} NodeMessage */

const stdK = 0.8e-7;

/** cusum mini k value (in hours^-1) */
const w = 1 / 6.66;

/** Default charge at peak power - see [battery university](https://batteryuniversity.com/article/bu-409-charging-lithium-ion) article */
const peakCharge = 80.5;

/** Charger efficiency */
const chargerEfficiency = 0.67;

/**
 * Round to nearest stdev below
 * @param {number} v
 */
const floorVal = (v, stdev = 0.5) =>
  Math.sign(v) * Math.floor(Math.abs(v) / stdev) * stdev;

/** @type {import("node-red").NodeInitializer} */
const nodeInit = (RED) => {
  /** @this {import("node-red").Node} */
  function Monitor(config) {
    // DEBUG
    // const fdata = `/tmp/data-${new Date().toISOString()}.csv`;
    // fs.appendFileSync(fdata, "timestamp,value\n");

    RED.nodes.createNode(this, config);
    const node = this;
    /** @type {number} target charge% */
    const maxCharge = config.cutoff || 85;

    /** @type {import("./types").Props} */
    let props;

    const initProps = () => {
      /** @type {import("./types").Props} */
      const p = {
        filter: new KalmanFilter(stdK), // our actual filter
        before: 0,
        cusum: 0,
        decaying: props ? props.decaying : false,
        energy: props ? props.energy : 0,
        battCap: props ? props.battCap : 0,
        maxPwr: props ? props.maxPwr : 0,
        timeout: props ? props.timeout : null,
        finishing: false,
        startTime: 0,
      };
      p.filter.init(0);
      return p;
    };

    props = initProps();
    node.log("Props initialized");

    const mainLoop = (
      /** @type {number} */ now,
      /** @type {number} */ timestep,
      /** @type {number} */ timeout = 5500,
      /** @type {(msg: NodeMessage | Array<NodeMessage | NodeMessage[] | null>) => void} */ nodeSend,
    ) => {
      if (props.timeout) {
        clearTimeout(props.timeout);
        props.timeout = null;
      }

      const [val, k] = props.filter.mean();
      if (val === null || k === null) return;

      const isSettled = (now - props.startTime) > 5 * 60e3; // min 5 minutes

      /**
       * Integrate over k, the inverse of the decay time constant
       * Assuming k ~ 1/1h, it gives us a value in normalized "time" (RC = 1h, 50% = 0.7h, 95% = 3h)
       * Returns true if we are full
       */
      const evalCusum = () => {
        if (!isSettled) return false;

        const thr = (maxCharge - peakCharge) / (100 - peakCharge);
        const maxCusum = -Math.log(1 - thr)

        /** k in hour^-1 */
        const kH = k * 3600;

        const prevCusum = props.cusum;
        /** dimensionless */
        const inc = (kH - w) * (timestep / 3600);
        props.cusum = Math.max(0, props.cusum + inc);
        if (inc > 0) props.cusum += w * (timestep / 3600); // compensate for threshold in our integral

        // save max power for later
        if (props.cusum === 0) props.maxPwr = val;

        if (props.cusum >= w / 60 && prevCusum < w / 60) {
          // threshold up
          props.decaying = true;
        } else if (props.cusum <= w / 60 && prevCusum > w / 60) {
          // threshold down
          props.decaying = false;
        }

        if (props.cusum > maxCusum && !props.finishing) {
          return true;
        }
        return false;
      };

      const fullCharge = evalCusum();
      if (fullCharge) {
        if (props.battCap === 0 && props.cusum >= 10) {
          // try to estimate battery capacity from max value and decay constant
          props.battCap =
            (props.maxPwr * chargerEfficiency) /
            ((100 - peakCharge) / 100);
        }
        node.log(
          `Scheduling one time OFF, ${val.toFixed(
            1
          )} / ${props.maxPwr.toFixed(1)} W, k=${(k * 3600).toFixed(2)}`
        );
        props.finishing = true;
        // wrap in timeout to avoid simultaneous read / write on some devices (Meross)
        setTimeout(() => {
          // props.finishing is reset on reset condition
          if (props.finishing)
            nodeSend([
              null,
              null,
              { payload: false }, // OFF payload
            ]);
        }, 2000);
        return;
      };

      const accel = isSettled ? k * (0 - val) : 0;

      if (val >= 0.05)
        if (props.before === 0 && !props.finishing) {
          // init our energy
          props.energy = 0;
          props.battCap = 0;
          props.before = now;
        } else {
          const dt = (now - props.before) / 1e3;
          props.energy += val * chargerEfficiency * dt;
          props.before = now;
        }

      /** in Wh */
      const nrg = props.energy / 3600;

      if (val < 0.05) {
        node.status({
          fill: "red",
          shape: "ring",
          text: `No input - last ${nrg.toFixed(1)} / ${props.battCap.toFixed(1) || "??"
            } Wh (est. cap.)`,
        });
      } else {
        // display acceleration, in Wh per hour^2, ie W/h, rounded to aribtrary stdev of 0.5 W/h
        const roundedAccel = floorVal((accel || 0) * 3600, 0.5);
        // display acceleration, in % per hour
        const dispAccel = (val && roundedAccel / val) || 0;
        let dir = "→";
        if (dispAccel < -1) dir = "↓";
        else if (dispAccel < -0.5) dir = "↘";
        if (dispAccel > 1) dir = "↑";
        else if (dispAccel > 0.5) dir = "↗";
        node.status({
          fill: props.decaying ? "yellow" : "green",
          shape: "ring",
          text: `${dir} ${val.toFixed(2)} W - total ${nrg.toFixed(1)} Wh`,
        });
      }

      nodeSend([
        { payload: val, topic: "value" }, // this is rate (Wh / h)
        {
          payload: 3600 * accel, // this is accel (Wh / h^2)
          topic: "rate",
        },
        null,
      ]);

      props.timeout = setTimeout(() => {
        // in timeout loop => predicts, evaluate cusum, pushResult
        const nextNow = performance.now();
        const ts1 = props.filter.predict(nextNow);
        mainLoop(nextNow, ts1, timeout, nodeSend);
      }, timeout);
    };

    node.on("input", (msg, send, done) => {
      const pv = Number(msg.payload);

      // Node-RED 0.x compat - https://nodered.org/docs/creating-nodes/node-js#sending-messages
      const nodeSend =
        send ||
        function () {
          node.send.apply(node, arguments);
        };

      if (pv !== null && !isNaN(pv) && isFinite(pv)) {
        // in value => predicts, check for reset (filter, cusum, not energy?), or correct, evaluate cusum, pushResult
        const now = performance.now();

        /** timestep */
        const ts = props.filter.predict(now);

        const [prevVal] = props.filter.mean();

        // check for reset condition
        // TODO: just detect for ON condition, otherwise use error detection (no need to resetState?)
        if (
          prevVal === null ||
          (prevVal + pv && !(prevVal * pv)) || // start or stop
          Math.abs((pv - prevVal) / pv) > 0.1 // 10% step triggers a reset
        ) {
          // reset
          node.log("Resetting filters");
          props.filter.resetState(pv);
          props.decaying = false;
          props.finishing = false;
          props.startTime = now;
          if (prevVal === 0 && pv) {
            props.before = 0;
            props.cusum = 0;
          }
        } else {
          // check for covariance reset condition (detect model error)
          const error = Math.abs(pv - prevVal);
          const noise = props.filter.state?.covariance[0][0] ** .5 + pwrStdev;
          if (error > 3 * noise) { // TODO cusum with 2 * noise ?
            props.filter.resetCovariance();
          }
        }

        props.filter.correct(pv, ts);
        mainLoop(now, ts, 5500, nodeSend);
      } else {
        node.status({ fill: "red", shape: "dot", text: "Bad input value" });
      }
      if (done) done();
    });

    node.on("close", () => {
      if (props.timeout) {
        clearTimeout(props.timeout);
        props.timeout = null;
      }
      props = initProps();
    });
  }

  RED.nodes.registerType("smartcharge-monitor", Monitor);
};

module.exports = nodeInit;
