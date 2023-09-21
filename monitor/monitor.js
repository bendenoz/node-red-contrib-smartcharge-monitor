const { IIRFilter } = require("./iir-filter");
const { KalmanFilter } = require("./kalman-filter");
const { SimpleKalmanFilter } = require("./simple-kalman-filter");
const { RollingDerivate } = require("./sg-derivate");

const fs = require("fs");

// Detection constants base on relative velocity

/** stdev (of relVel) */
const sigma = 1;
/** relative tolerance */
const w = 0.0 * sigma;

/** Too high velocity, ignore "jumps" */
const maxScore = 8; // aka 800%

/** @type {import("node-red").NodeInitializer} */
const nodeInit = (RED) => {
  /** @this {import("node-red").Node} */
  function Monitor(config) {
    // DEBUG
    const fdata = `/tmp/data-${new Date().toISOString()}.csv`;
    fs.appendFileSync(fdata, "timestamp,value\n");

    RED.nodes.createNode(this, config);
    const node = this;
    /** @type {number} */
    const pvStdDev = config.stddev || 0.05;
    /** @type {number} timestep in seconds */
    const timestep = config.timestep || 20;
    /** @type {number} relative average */
    const avg = config.avg || -0.5;
    /** @type {number} relVel cusum threshold */
    const t = config.rateMinutes || 6; // %rate . minutes

    /** @type {import("./types").Props} */
    let props;

    const initProps = () => {
      /** @type {import("./types").Props} */
      const p = {
        fastFilter: new KalmanFilter(0.05, timestep, 160), // our actual filter
        slowFilter: new KalmanFilter(0.05, timestep, 4800), // used for display
        velocity: new IIRFilter(12), // used for display
        accel: new RollingDerivate(20), // in W/s^2, not used
        before: 0,
        cusum: 0,
        energy: props ? props.energy : 0,
      };
      return p;
    };

    props = initProps();

    node.on("input", (msg, send, done) => {
      const pv = Number(msg.payload);

      if (pv !== null && !isNaN(pv) && isFinite(pv)) {
        const now = performance.now();

        fs.appendFileSync(fdata, `${now},${pv}\n`);

        const lastValue = props.slowFilter.mean()[0];
        props.slowFilter.push(pv);
        props.fastFilter.push(pv);

        const [valSlow, spdSlow] = props.slowFilter.mean();
        const [valFast, spdFast] = props.fastFilter.mean();

        if (
          pv &&
          pv > 2 * pvStdDev &&
          (lastValue === null || lastValue < 2 * pvStdDev)
        ) {
          // we reset total energy when signal is detected
          props.energy = 0;
        }

        /** @type {import("@node-red/registry").NodeMessage | null} */
        let trigger = null;
        if (
          props.before &&
          valSlow !== null &&
          valFast !== null &&
          spdSlow !== null &&
          spdFast !== null
        ) {
          const dt = (now - props.before) / 1e3;

          const relVel = (valFast && ((spdFast || 0) * 3600) / valFast) || 0;
          const zScore = (relVel - avg) / sigma;
          if (Math.abs(zScore) < maxScore) {
            props.cusum = Math.max(
              0,
              props.cusum - ((zScore + w) * timestep) / 60
            );
            if (props.cusum > t) {
              props.cusum = 0;
              // wrap in timeout to avoid simultaneous read / write on some devices (meross)
              setTimeout(() => {
                send([
                  null,
                  null,
                  { payload: false }, // OFF payload
                ]);
              }, 5000);
            }
            if (props.slowFilter.count() > 6) {
              props.velocity.push(spdFast * 0.6 + spdSlow * 0.4);
            } else {
              // or just ignore the first few samples
              props.velocity.push(0);
            }
          } else {
            // step change detected, reset everything
            props.cusum = 0;
            props.slowFilter.resetCovariance(pv);
            props.velocity.reset();
          }

          // update cumul
          props.energy += valSlow * dt;
          props.accel.push(props.velocity.mean() || 0);
        }
        props.before = now;

        const v = props.slowFilter.mean()[0] || 0;
        const st = props.slowFilter.stddev() || 0;
        const k = props.slowFilter.K[0];
        const nrg = props.energy / 3600;

        // display velocity, in % per hour
        const relVel = (v && ((props.velocity.mean() || 0) * 3600) / v) || 0;
        const dispVel =
          Math.sign(relVel) * Math.floor(Math.abs(relVel) / 0.01) * 0.01;
        let dir = "→";
        if (dispVel < -1) dir = "↓";
        else if (dispVel < -0.5) dir = "↘";
        if (dispVel > 1) dir = "↑";
        else if (dispVel > 0.5) dir = "↗";

        if (v < 2 * pvStdDev) {
          node.status({
            fill: "red",
            shape: "ring",
            text: `No input - last ${nrg.toFixed(1)} Wh`,
          });
          props = initProps();
        } else {
          node.status({
            fill: props.cusum < 0.24 * t ? "green" : "yellow",
            shape: "ring",
            text: `${dir} ${v.toFixed(2)} W - total ${nrg.toFixed(1)} Wh`,
          });
        }

        const validSpeed = props.slowFilter.count() >= 2;

        send([
          { payload: v, topic: "value" },
          validSpeed ? { payload: dispVel, topic: "rate" } : null,
          null,
          // { payload: st, topic: "stddev" },
          // validSpeed ? { payload: props.accel.sg(), topic: "accel" } : null,
        ]);
      } else {
        node.status({ fill: "red", shape: "dot", text: "Bad input value" });
      }
      done();
    });

    node.on("close", () => {
      props = initProps();
    });
  }

  RED.nodes.registerType("smartcharge-monitor", Monitor);
};

module.exports = nodeInit;
