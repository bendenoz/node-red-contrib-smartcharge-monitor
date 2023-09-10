const { IIRFilter } = require("./iir-filter");
const { KalmanFilter } = require("./kalman-filter");
const { SimpleKalmanFilter } = require("./simple-kalman-filter");
const { RollingDerivate } = require("./sg-derivate");

// Detection constants base on relative velocity

/** stdev */
const sigma = 0.98;
/** relative tolerance */
const w = 0.15 * sigma;
/** relative average */
const avg = -0.5;
/** relative threshold */
const t = 15;
/** Too high velocity, ignore */
const maxScore = 5;

/** @type {import("node-red").NodeInitializer} */
const nodeInit = (RED) => {
  /** @this {import("node-red").Node} */
  function Monitor(config) {
    RED.nodes.createNode(this, config);
    const node = this;
    const pvStdDev = config.stddev || 0.05;

    /** @type {import("./types").Props} */
    let props;

    const initProps = () => {
      /** @type {import("./types").Props} */
      const p = {
        slowFilter: new KalmanFilter(0.05, 20, 3600),
        fastFilter: new KalmanFilter(0.05, 20, 60),
        velocity: new IIRFilter(15),
        accel: new RollingDerivate(20),
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

        const lastValue = props.slowFilter.mean()[0];
        props.slowFilter.push(pv);
        props.fastFilter.push(pv); // todo, delay slow by one sample

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

          // const zScore = Math.abs((pv - value) / pvStdDev);
          // if (zScore > 10 || (zScore > 3 && props.slowFilter.count() > 8)) {
          //   props.slowFilter.resetCovariance(pv);
          // }

          const relVel = (valFast && ((spdFast || 0) * 3600) / valFast) || 0;
          const zScore = (relVel - avg) / sigma;
          if (Math.abs(zScore) < maxScore) {
            props.cusum = Math.max(0, props.cusum - zScore - w);
            if (props.cusum > t) {
              props.cusum = 0;
              // wrap in timeout to avoid read / write on some devices (meross)
              setTimeout(() => {
                send([
                  null,
                  null,
                  null,
                  null,
                  { payload: false }, // OFF payload
                ]);
              }, 5000);
            }
            if (props.slowFilter.count() > 1) {
              props.velocity.push(spdFast * 0.6 + spdSlow * 0.4);
            } else {
              props.velocity.push(0);
            }
          } else {
            props.slowFilter.resetCovariance(pv);
            // props.slowFilter.push(pv);
          }

          // update cumul
          props.energy += valSlow * dt;
          props.accel.push(props.velocity.mean() || 0);

          // now try to be smart and analyze the slope if we have enough data
          // FIXME - Move to another node
          // const v = props.value.mean();
          // if (props.value.n > 15 && v) {
          //   /** Rate in % per hour */
          //   const testRate =
          //     (100 * (3600 * (props.longRate.mean() || 0))) / v;
          //   props.cusum =
          //     testRate > -800 && testRate < -90
          //       ? props.cusum + (testRate - -90)
          //       : Math.min(0, props.cusum + 10);
          //   if (props.cusum < -120) {
          //     trigger = { payload: false }; // OFF payload
          //     // props = initProps(); // TBC
          //   }
          // }
        }
        props.before = now;

        const v = props.slowFilter.mean()[0] || 0;
        const st = props.slowFilter.stddev() || 0;
        const k = props.slowFilter.K[0];
        const nrg = props.energy / 3600;
        if (v < 2 * pvStdDev) {
          node.status({
            fill: "red",
            shape: "ring",
            text: `No input - last ${nrg.toFixed(1)} Wh`,
          });
          props = initProps();
        } else {
          node.status({
            fill: "green",
            shape: "ring",
            text: `${v.toFixed(2)} W (±${st.toFixed(2)}) - total ${nrg.toFixed(
              1
            )} Wh - K ${k.toFixed(3)}`,
          });
        }

        const validSpeed = props.slowFilter.count() >= 2;

        send([
          { payload: v, topic: "value" },
          { payload: st, topic: "stddev" },
          validSpeed
            ? { payload: props.velocity.mean() || 0, topic: "rate" }
            : null,
          validSpeed ? { payload: props.accel.sg(), topic: "accel" } : null,
          null,
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
