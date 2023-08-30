// @ts-check

const { IIRFilter } = require("./iir-filter");

/** @type {import("node-red").NodeInitializer} */
const nodeInit = (RED) => {
  /** @this {import("node-red").Node} */
  function Monitor(config) {
    RED.nodes.createNode(this, config);
    const node = this;

    const initProps = () => {
      /** @type {import("./types").Props} */
      const p = {
        value: new IIRFilter(9),
        longRate: new IIRFilter(30),
        shortRate: new IIRFilter(12),
        lastSampleTime: 0,
        cusum: 0,
      };
      return p;
    };

    let props = initProps();

    node.on("input", (msg, send, done) => {
      const pv = Number(msg.payload);

      if (pv !== null && !isNaN(pv) && isFinite(pv)) {
        const now = Date.now();

        const lastValue = props.value.mean();
        props.value.push(pv);
        const value = props.value.mean();

        let rate = 0;
        let accel = 0;
        /** @type {import("@node-red/registry").NodeMessage | null} */
        let trigger = null;
        if (props.lastSampleTime && value !== null && lastValue !== null) {
          const dt = (now - props.lastSampleTime) / 1e3;

          // test for reset condition
          const count = props.value.count();
          if (count > 6) {
            // to get meanValueTime, we assume dt is constant accross all samples
            // TODO: Add warning if not
            const meanValueTime = ((count - 1) / 2 + 1) * dt;
            const predictedValue =
              lastValue + meanValueTime * (props.longRate.mean() || 0);
            const zScore = Math.abs(
              (pv - predictedValue) / (config.stddev || 0.05)
            );
            if (zScore > 5) {
              props = initProps();
              props.value.push(pv);
            }
          }

          if (props.lastSampleTime) {
            /** in Watt/sec */
            const currentRate = (value - lastValue) / dt;
            props.longRate.push(currentRate);
            props.shortRate.push(currentRate);

            rate = props.shortRate.mean() || 0;
            accel = (rate - (props.longRate.mean() || 0)) / (12 * dt); // in Watt/sec^2

            // now try to be smart and analyze the slope if we have enough data
            // FIXME - Move to another node
            const v = props.value.mean();
            if (props.value.n > 15 && v) {
              /** Rate in % per hour */
              const testRate = (100 * 3600 * (props.longRate.mean() || 0)) / v;
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
        props.lastSampleTime = now;

        send([
          { payload: props.value.mean() || 0 },
          { payload: props.value.stddev() || 0 },
          { payload: rate },
          { payload: accel },
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
