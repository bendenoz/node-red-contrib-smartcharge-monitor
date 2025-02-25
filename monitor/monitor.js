const { KalmanFilter, pwrStdev } = require("./kalman-filter");
const { State } = require("kalman-filter");

/** @typedef {import("node-red").NodeMessage} NodeMessage */

const stdK = 0.77e-7;

/** cusum mini k value (in hours^-1) */
const w = 1 / 6.66;

/** Default charge at peak power - see [battery university](https://batteryuniversity.com/article/bu-409-charging-lithium-ion) article */
const peakCharge = 80.5;

/** Charger efficiency */
const chargerEfficiency = 0.67;

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

    const initProps = (/** @type {number} */ now) => {
      /** @type {import("./types").Props} */
      const p = {
        kfSlow: new KalmanFilter(stdK), // display filter
        kfFast: new KalmanFilter(400 * stdK), // cusum (detection) filter
        lastPush: 0,
        cusum: 0,
        triggerCusum: 0,
        energy: props ? props.energy : 0,
        battCap: props ? props.battCap : 0,
        maxPwr: props ? props.maxPwr : 0,
        timeout: props ? props.timeout : null,
        finishing: false,
        startTime: 0,
      };
      p.kfSlow.init(0, now);
      p.kfFast.init(0, now);
      return p;
    };

    props = initProps(Date.now());
    node.log("Props initialized");

    const evalCusum = (/** @type {number} */ timestep) => {
      const [, kSlow] = props.kfSlow.mean();
      const [, kFast] = props.kfFast.mean();
      if (kFast === null || kSlow === null) return;

      /** k in hour^-1 */
      const kH = kFast * 3600;

      const inc = (kH - kSlow * 3600 - w) * (timestep / 60);
      props.cusum = Math.max(0, props.cusum + inc);
      if (inc > 0) props.cusum += w * (timestep / 60);

      const triggerInc = (kH - w) * (timestep / 60);
      props.triggerCusum = Math.max(0, props.triggerCusum + triggerInc);
      if (triggerInc > 0) props.triggerCusum += w * (timestep / 60);
      else props.triggerCusum = 0;
    };

    const evalTrigger = (
      /** @type {(msg: NodeMessage | Array<NodeMessage | NodeMessage[] | null>) => void} */ nodeSend
    ) => {
      const threshold = (100 + 5 - maxCharge) / (100 + 5 - peakCharge); // 5 to compensate for actual end of charge @90% of saturation (TBC)
      const [val, k] = props.kfSlow.mean();
      if (val === null || k === null) return;

      if (
        props.triggerCusum > 2 &&
        props.maxPwr &&
        val <= props.maxPwr * threshold
      ) {
        node.log("Triggered");
        node.log(
          `Scheduling one time OFF, ${val.toFixed(1)} / ${props.maxPwr.toFixed(
            1
          )} W, k=${(k * 3600).toFixed(2)}`
        );
        props.finishing = true;
        // wrap in timeout to avoid simultaneous read / write on some devices (Meross)
        setTimeout(() => {
          if (props.finishing) {
            node.log("Sending OFF");
            nodeSend([
              null,
              null,
              { payload: false }, // OFF payload
            ]);
            props.finishing = false;
            props.cusum = 0;
            props.triggerCusum = 0;
            // more like props = initProps(); ?
            // + keep looping anyway?
          }
        }, 2000);
        return true;
      }
      return false;
    };

    const pushOut = (
      /** @type {number} */ now,
      /** @type {(msg: NodeMessage | Array<NodeMessage | NodeMessage[] | null>) => void} */ nodeSend
    ) => {
      const [val, k] = props.kfSlow.mean();
      const [, kFast] = props.kfFast.mean();
      if (val === null || k === null || kFast === null) return;

      const dt = (now - props.lastPush) / 1e3;
      props.energy += val * chargerEfficiency * dt;
      props.lastPush = now;

      /** in Wh */
      const nrg = props.energy / 3600;

      const isSettled = now - props.startTime > 1 * 60e3; // min 1 minutes
      const accel = isSettled ? k * 3600 * (0 - val) : null; // (Wh / h^2)

      const hasDecay = kFast * 3600 > w;
      // save for later
      if (!hasDecay) props.maxPwr = val;

      if (val < 0.05) {
        node.status({
          fill: "red",
          shape: "ring",
          text: `No input - last ${nrg.toFixed(1)}  Wh`,
        });
      } else {
        const dispAccel = -k * 3600 || 0;
        let dir = "→";
        if (dispAccel < -1) dir = "↓";
        else if (dispAccel < -0.5) dir = "↘";
        if (dispAccel > 1) dir = "↑";
        else if (dispAccel > 0.5) dir = "↗";
        node.status({
          fill: hasDecay ? "yellow" : "green",
          shape: "ring",
          text: `${dir} ${val.toFixed(2)} W - total ${nrg.toFixed(1)} Wh`,
        });
      }

      nodeSend([
        { payload: val, topic: "value" }, // this is rate (Wh / h)
        {
          payload: accel, // this is acceleration (Wh / h^2)
          topic: "rate",
        },
        null,
      ]);
    };

    const scheduleUpdate = (
      /** @type {(msg: NodeMessage | Array<NodeMessage | NodeMessage[] | null>) => void} */ nodeSend,
      /** @type {number} */ timeout = 5500
    ) => {
      props.timeout = setTimeout(() => {
        const now = Date.now();
        const timestep = (now - props.kfFast.stateTS) / 1000;
        props.kfSlow.predict(now, props.cusum > 0.2 ? 30 * props.cusum : 1);
        props.kfFast.predict(now);
        pushOut(now, nodeSend);
        evalCusum(timestep);
        if (!evalTrigger(nodeSend)) scheduleUpdate(nodeSend);
      }, timeout);
    };

    node.on("input", (msg, send, done) => {
      const pv = Number(msg.payload);
      if (props.finishing) {
        done();
        return;
      }

      if (props.timeout) {
        clearTimeout(props.timeout);
        props.timeout = null;
      }

      // Node-RED 0.x compat - https://nodered.org/docs/creating-nodes/node-js#sending-messages
      const nodeSend =
        send ||
        function () {
          node.send.apply(node, arguments);
        };

      if (pv !== null && !isNaN(pv) && isFinite(pv)) {
        // in value => predicts, check for reset (filter, cusum, not energy?), or correct, evaluate cusum, pushResult
        const now = Date.now();

        const reInit = () => {
          props.kfSlow.init(pv, now);
          props.kfFast.init(pv, now);
          props.lastPush = 0;
          props.cusum = 0;
          props.triggerCusum = 0;
          props.startTime = now;
          pushOut(now, nodeSend);
          if (done) done();
        };

        /** timestep (before predict !) */
        const timestep = (now - props.kfFast.stateTS) / 1000;
        props.kfSlow.predict(now, props.cusum > 0.2 ? 30 * props.cusum : 1);
        props.kfFast.predict(now);

        const [prevVal] = props.kfSlow.mean();

        // Check for reset conditions

        // first detect start or stop
        if (prevVal === null || (prevVal + pv && !(prevVal * pv))) {
          if (pv) {
            node.log("Start detected");
            props.energy = 0;
            props.battCap = 0;
          } else {
            node.log("Stop detected");
          }
          reInit();
          return;
        }

        // then detect noise
        const error = prevVal === null ? 0 : Math.abs(pv - prevVal);
        const noise = props.kfSlow.state?.covariance[0][0] ** 0.5 + pwrStdev;
        const noiseReset = error > 2.5 * noise;
        if (noiseReset) {
          node.log("Noise reset");
          props.kfSlow.state =
            props.kfFast.state && new State(props.kfFast.state);
        }

        props.kfSlow.correct(pv, now);
        props.kfFast.correct(pv, now);

        // finally check for incoherent k values
        const [, correctedKFast] = props.kfFast.mean();
        if (correctedKFast !== null) {
          // reset on time constant less than 10 minutes
          if (correctedKFast * 60 > 10 ** -1) {
            node.log("reinit on time constant too high");
            reInit();
            return;
          }

          // reset on time constant less than -20 minutes
          if (correctedKFast * 60 < -(20 ** -1)) {
            node.log("reinit on high negative time constant");
            reInit();
            return;
          }
        }

        pushOut(now, nodeSend);
        evalCusum(timestep);
        if (!evalTrigger(nodeSend)) scheduleUpdate(nodeSend); // TODO: evalTrigger should call scheduleUpdate ?
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
