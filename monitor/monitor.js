const { KalmanFilter } = require("./kalman-filter");
const { State } = require("kalman-filter");
const { matMul, transpose } = require("simple-linalg");

/** @typedef {import("node-red").NodeMessage} NodeMessage */

const stdK = 0.77e-7;

/** cusum mini k value (in hours^-1) */
const w = 1 / 6.66;

const p11 = 0.995; // ~ 1 - deltaT / tauSlow
const p22 = 0.925; // ~ 1 - deltaT / tauFast
const Pi = [
  [p11, 1 - p11], // from slow -> [slow, fast]
  [1 - p22, p22], // from fast -> [slow, fast]
];

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

    /** @type {number} peak charge% */
    const peakCharge = config.peakCharge || 80;

    /** @type {number} charger efficiency */
    const chargerEfficiency = config.chargerEfficiency || 0.72;

    /** @type {number} power standard deviation */
    const pwrStdev = config.pwrStdev || 0.15;

    /** @type {import("./types").Props} */
    let props;

    const initProps = (/** @type {number} */ now) => {
      /** @type {import("./types").Props} */
      const p = {
        kfSlow: new KalmanFilter(stdK, pwrStdev), // display filter
        kfFast: new KalmanFilter(50 * stdK, pwrStdev), // cusum (detection) filter
        mu: [1, 0], // fast at first
        lastPush: now,
        triggerCusum: 0,
        energy: props ? props.energy : 0,
        battCap: props ? props.battCap : 0,
        maxPwr: props ? props.maxPwr : 0,
        timeout: props ? props.timeout : null,
        finishing: false,
        startTime: now,
      };
      p.kfSlow.init(0, now);
      p.kfFast.init(0, now);
      return p;
    };

    props = initProps(Date.now());
    node.log("Props initialized");

    const evalCusum = (/** @type {number} in seconds */ timestep) => {
      const [, kSlow] = props.kfSlow.mean();
      const [, kFast] = props.kfFast.mean();
      if (kFast === null || kSlow === null) return;

      const kFuse = props.mu[0] * kSlow + props.mu[1] * kFast;

      /** k in hour^-1 */
      const kH = kFuse * 3600;

      const triggerInc = (kH - w) * (timestep / 60); // again, hour^-1 ⋅ min
      props.triggerCusum = Math.max(0, props.triggerCusum + triggerInc);
      if (triggerInc > 0) props.triggerCusum += w * (timestep / 60);
      else props.triggerCusum = 0;
    };

    const getFused = () => {
      const [valSlow, kSlow] = props.kfSlow.mean();
      const [valFast, kFast] = props.kfFast.mean();
      if (
        valSlow === null ||
        kSlow === null ||
        valFast === null ||
        kFast === null
      )
        return [null, null];

      const val = props.mu[0] * valSlow + props.mu[1] * valFast;
      const k = props.mu[0] * kSlow + props.mu[1] * kFast;
      return [val, k];
    };

    const evalTrigger = (
      /** @type {(msg: NodeMessage | Array<NodeMessage | NodeMessage[] | null>) => void} */ nodeSend
    ) => {
      const threshold = (maxCharge - peakCharge) / (100 - peakCharge); // 5 to compensate for actual end of charge @90% of saturation (TBC)
      const [val, k] = getFused();
      if (val === null || k === null) return false;

      if (
        props.triggerCusum > 2 && // assumes 2 mins at w - FIXME constant
        props.maxPwr &&
        val <= props.maxPwr * (1 - 0.9 * threshold) // assumes 100% at 10% of max power - FIXME constant
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
              { payload: 0 }, // optimistic zero
              { payload: 0 },
              { payload: false }, // OFF payload
            ]);
            props.finishing = false;
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
      const [val, k] = getFused();
      if (val === null || k === null) return false;

      const dt = (now - props.lastPush) / 1e3;
      props.energy += val * chargerEfficiency * dt;
      props.lastPush = now;

      /** in Wh */
      const nrg = props.energy / 3600;

      const accel = k * 3600 * (0 - val);

      const hasDecay = props.triggerCusum > w * 1;
      // save for later
      if (k * 3600 < w) props.maxPwr = val; // !hasDecay ?

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
        props.kfSlow.predict(now);
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
          props.mu = [1, 0];
          props.lastPush = now;
          props.triggerCusum = 0;
          props.startTime = now;
          pushOut(now, nodeSend);
          if (done) done();
        };

        /** timestep, in second (before predict !) */
        const timestep = (now - props.kfFast.stateTS) / 1000;
        props.kfSlow.predict(now);
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

        props.kfSlow.correct(pv, now);
        props.kfFast.correct(pv, now);

        const mahalanobis = /** @type {number} */ (props.kfFast.nis) ** 0.5;
        if (mahalanobis > 3 && mahalanobis >= 0.2 * pv) {
          // reset !
          reInit();
          return;
        } else {
          // go MMAE (Multiple Model Adaptive Estimation)
          const logL = /** @type {number[]} */ ([
            props.kfSlow.logL ?? Number.NEGATIVE_INFINITY,
            props.kfFast.logL ?? Number.NEGATIVE_INFINITY,
          ]);

          const muPrior =
            /** @type {[number, number]} */
            (
              matMul(transpose(Pi), [[props.mu[0]], [props.mu[1]]]).map(
                (v) => v[0]
              )
            );
          // normalize and update mu (numerically stable, guard against under/overflow)
          const maxLogL = Math.max(...logL);
          let weights =
            /** @type {[number, number]} */
            (muPrior.map((m, i) => m * Math.exp(logL[i] - maxLogL)));
          let muSum = weights.reduce((a, b) => a + b, 0);
          if (!isFinite(muSum) || muSum === 0) {
            // fallback to uniform prior if weights collapsed
            weights = [0.5, 0.5];
            muSum = 1;
          }
          props.mu = /** @type {[number, number]} */ (
            weights.map((w) => w / muSum)
          );
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
      props = initProps(Date.now());
    });
  }

  RED.nodes.registerType("smartcharge-monitor", Monitor);
};

module.exports = nodeInit;
