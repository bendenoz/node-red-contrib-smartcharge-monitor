// @ts-check

const { IIRFilter } = require("./iir-filter");

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
        value: new IIRFilter(20),
        longRate: new IIRFilter(30), // FIXME - 10 minutes, not 30 samples
        shortRate: new IIRFilter(20), // 0.4 * 30 ?
        before: 0,
        cusum: 0,
        energy: props ? props.energy:  0,
      };
      return p;
    };

    props = initProps();

    node.on("input", (msg, send, done) => {
      const pv = Number(msg.payload);

      if (pv !== null && !isNaN(pv) && isFinite(pv)) {
        const now = performance.now();

        const lastValue = props.value.mean();
        props.value.push(pv);
        const value = props.value.mean();

        let rate = 0;
        let accel = 0;
        /** @type {import("@node-red/registry").NodeMessage | null} */
        let trigger = null;
        if (props.before && value !== null && lastValue !== null) {
          const dt = (now - props.before) / 1e3;

          if (value > 2*pvStdDev && lastValue < 2 * pvStdDev) {
            // we reset total energy when signal is detected 
            props.energy = 0;
          }

          // test for reset condition
          // FIXME constant (0.25 * 30 ?)
          /*
          if (props.longRate.count() > 8) {
            // to get meanValueTime, we assume dt is constant accross all samples
            // TODO: Add warning if not
            const meanValueTime = props.value.delay() * dt;
            const predictedValue =
              lastValue + meanValueTime * (props.longRate.mean() || 0);
            const zScore = Math.abs((pv - predictedValue) / pvStdDev);
            if (zScore > 5) {
              props = initProps();
              props.value.push(pv);
            }
          }
          */

          if (props.before) {
            // update cumul
            props.energy += value * dt;

            /** in Watt/sec */
            const currentRate = (value - lastValue) / dt;
            props.longRate.push(currentRate);
            props.shortRate.push(currentRate);

            rate = props.longRate.mean() || 0;
            /** mean value delta time */
            const mvdt =
              Math.max(1, props.longRate.delay() - props.shortRate.delay()) *
              dt;
            accel = ((props.shortRate.mean() || 0) - rate) / mvdt; // in Watt/sec^2

            // now try to be smart and analyze the slope if we have enough data
            // FIXME - Move to another node
            const v = props.value.mean();
            if (props.value.n > 15 && v) {
              /** Rate in % per hour */
              const testRate =
                (100 * (3600 * (props.longRate.mean() || 0))) / v;
              props.cusum =
                testRate > -800 && testRate < -90
                  ? props.cusum + (testRate - -90)
                  : Math.min(0, props.cusum + 10);
              if (props.cusum < -120) {
                trigger = { payload: false }; // OFF payload
                // props = initProps(); // TBC
              }
            }
          }
        }
        props.before = now;

        const v = props.value.mean() || 0;
        const st = props.value.stddev() || 0;
        if (v < 2 * pvStdDev) {
          node.status({ fill: "red", shape: "ring", text: "No input" });
          props = initProps();
        } else {
          node.status({
            fill: "green",
            shape: "ring",
            text: `${v.toFixed(2)} W (±${st.toFixed(2)}) - total ${(
              props.energy / 3600
            ).toFixed(1)} Wh`,
          });
        }

        send([
          { payload: v, topic: "value" },
          { payload: st, topic: "stddev" },
          { payload: rate, topic: "rate" },
          { payload: accel, topic: "accel" },
          trigger,
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
