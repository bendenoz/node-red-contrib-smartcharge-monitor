const { performance } = require("perf_hooks"); // not needed for node > ??

const { KalmanFilter } = require("./kalman-filter");

// const fs = require("fs");

const stdK = 1e-6;

/** cusum mini k value (in hours^-1) */
const w = 1 / 6.66;

/** Default charge at peak power - see [battery university](https://batteryuniversity.com/article/bu-409-charging-lithium-ion) article */
const peakCharge = 84.9;

/** Charger efficiency */
const chargerEfficiency = 0.8;

/** noise distribution around 1.5 cusum minutes */
const distrib = (x, mean = 1.5, sigma = 0.3) => {
  var exponent = -((x - mean) ** 2) / (2 * sigma ** 2);
  return Math.exp(exponent);
};

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

    node.on("input", (msg, send, done) => {
      const pv = Number(msg.payload);

      // Node-RED 0.x compat - https://nodered.org/docs/creating-nodes/node-js#sending-messages
      const nodeSend =
        send ||
        function () {
          node.send.apply(node, arguments);
        };

      /**
       * Integrate over k, the inverse of the decay time constant
       * Assuming k ~ 1/1h, it gives us a value in normalized "time" (RC = 1h, 50% = 0.7h, 95% = 3h)
       * @param {number} timestep in seconds
       * @param {number} now
       */
      const evalCusum = (timestep, now) => {
        const [val, k] = props.filter.mean();
        if (k === null || val === null) return false;


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

        if (props.filter.state) {
          if (props.cusum >= w / 60 && prevCusum < w / 60) {
            // threshold up
            props.decaying = true;
          } else if (props.cusum <= w / 60 && prevCusum > w / 60) {
            // threshold down
            props.decaying = false;
          }
        }

        if (
          props.decaying &&
          !props.finishing &&
          (now - props.startTime) > 5 * 60e3 && // min 5 minutes
          props.cusum > maxCusum
        ) {
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
          return true;
        }
        return false;
      };

      const mainLoop = (
        /** @type {number} */ now,
        /** @type {number} */ timestep,
        timeout = 5000
      ) => {
        if (props.timeout) {
          clearTimeout(props.timeout);
          props.timeout = null;
        }

        const fullCharge = evalCusum(timestep, now);
        if (fullCharge) return;

        const [val, k] = props.filter.mean();
        if (val === null || k === null) return;
        const accel = ((now - props.startTime) > 5 * 60e3) ? k * (0 - val) : 0;

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
          mainLoop(nextNow, ts1);
        }, timeout);
      };

      if (pv !== null && !isNaN(pv) && isFinite(pv)) {
        // in value => predicts, check for reset (filter, cusum, not energy?), or correct, evaluate cusum, pushResult

        const now = performance.now();

        /** timestep */
        const ts = props.filter.predict(now);

        const [prevVal] = props.filter.mean();

        // check for reset condition
        if (
          prevVal === null ||
          (prevVal + pv && !(prevVal * pv)) || // start or stop
          Math.abs((pv - prevVal) / pv) > 0.1 // 10% step triggers a reset
        ) {
          // reset
          node.log("Resetting filters");
          props.filter.resetCovariance(pv);
          props.decaying = false;
          props.finishing = false;
          props.startTime = now;
          if (prevVal === 0 && pv) {
            props.before = 0;
            props.cusum = 0;
          }
        } else {
          props.filter.correct(pv, ts);
        }
        mainLoop(now, ts, 5500);
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
