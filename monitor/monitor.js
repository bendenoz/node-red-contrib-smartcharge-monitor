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
        count: 0,
        value: new IIRFilter(9),
        longRate: new IIRFilter(30),
        shortRate: new IIRFilter(12),
        lastSampleTime: 0,
      };
      return p;
    };

    let props = initProps();

    node.on("input", (msg, send, done) => {
      if (typeof msg.payload === "number") props.pv = msg.payload;

      if (props.pv && !isNaN(props.pv) && isFinite(props.pv)) {
        const now = Date.now();
        const lastValue = props.value.mean();
        props.value.push(props.pv);
        const value = props.value.mean();

        let rate = 0;
        let accel = 0;
        if (props.lastSampleTime) {
          const dt = (now - props.lastSampleTime) / 1e3;
          /** in Watt/sec */
          const currentRate = (value - lastValue) / dt;
          props.longRate.push(currentRate);
          props.shortRate.push(currentRate);
          rate = props.longRate.mean();
          accel = (props.shortRate.mean() - rate) / (12 * dt); // in Watt/sec^2
        }
        props.lastSampleTime = now;
        send([
          { payload: value },
          { payload: props.value.stddev() },
          { payload: rate },
          { payload: accel },
        ]);
      } else {
        node.status({ fill: "red", shape: "dot", text: "Bad PV" });
      }
      done();
    });
  }

  RED.nodes.registerType("smartcharge-monitor", Monitor);
};

module.exports = nodeInit;
