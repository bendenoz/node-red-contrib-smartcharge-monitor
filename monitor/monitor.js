const { performance } = require("perf_hooks"); // not needed for node > ??

const { IIRFilter } = require("./iir-filter");
const { KalmanFilter } = require("./kalman-filter");
const { SimpleKalmanFilter } = require("./simple-kalman-filter");
const { RollingDerivate } = require("./sg-derivate");

// const fs = require("fs");

// Detection constants base on relative velocity

/** cusum min k value*/
const w = 0.5;

/** Cut-off voltage capacity - see [battery university](https://batteryuniversity.com/article/bu-409-charging-lithium-ion) article */
const cutOffCap = 84.5;

/** Charger efficiency */
const chargerEfficiency = 0.8;

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
        fastFilter: new KalmanFilter(0.003), // our actual filter
        slowFilter: new KalmanFilter(0.0002), // used for display
        acceleration: new IIRFilter(12), // (not) used for display
        before: 0,
        cusum: 0,
        energy: props ? props.energy : 0,
        battCap: props ? props.battCap : 0,
        timeout: null,
      };
      p.fastFilter.init(0);
      p.slowFilter.init(0);
      return p;
    };

    props = initProps();

    node.on("input", (msg, send, done) => {
      const pv = Number(msg.payload);

      // Node-RED 0.x compat - https://nodered.org/docs/creating-nodes/node-js#sending-messages
      const nodeSend =
        send ||
        function () {
          node.send.apply(node, arguments);
        };

      /**
       * Integrate over the inverse of the decay time constant
       * Assuming 80% SOC reached when decay is detected, this gives us
       * a good approximation the state of charge between 82% - 95%
       * @param {number} timestep
       */
      const evalCusum = (timestep) => {
        const [valFast, , kFast] = props.fastFilter.mean();
        if (kFast === null || valFast === null) return false;

        const t = Math.log(
          1 / (1 - (maxCharge - cutOffCap) / (100 - cutOffCap))
        );
        /** k in hour^-1 */
        const kH = kFast * 3600;

        const inc = (kH - w) * (timestep / 3600);
        props.cusum = Math.max(0, props.cusum + inc);
        if (inc > 0) props.cusum += w * (timestep / 3600); // compensate for threshold in our integral

        // node.log(
        //   JSON.stringify({
        //     cusum: props.cusum,
        //     t,
        //     kFast: kH,
        //     valFast,
        //   })
        // );
        if (props.cusum > t) {
          // try to estimate battery capacity
          if (props.battCap === 0) {
            // cusum is t for a k of 1
            const remainChargePct =
              (Math.exp(-props.cusum) * (100 - cutOffCap)) / 100;
            // deduce capacity from remaining charge, current value and decay constant
            if (remainChargePct > 0)
              // FIXME - Use average kH ?
              props.battCap =
                (valFast * chargerEfficiency) / kH / remainChargePct;
          }
          node.log("Scheduling one time OFF");
          // wrap in timeout to avoid simultaneous read / write on some devices (meross)
          setTimeout(() => {
            props.cusum = 0;
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

        const fullCharge = evalCusum(timestep);
        if (fullCharge) return;

        const [, accelSlow] = props.slowFilter.mean();
        const [valFast] = props.fastFilter.mean();
        if (valFast === null || accelSlow === null) return;

        if (valFast >= 0.05)
          if (props.before === 0) {
            // init our energy
            props.energy = 0;
            props.battCap = 0;
            props.before = now;
          } else {
            const dt = (now - props.before) / 1e3;
            props.energy += valFast * chargerEfficiency * dt;
            props.before = now;
          }

        /** in Wh */
        const nrg = props.energy / 3600;

        if (valFast < 0.05) {
          node.status({
            fill: "red",
            shape: "ring",
            text: `No input - last ${nrg.toFixed(1)} / ${
              props.battCap.toFixed(1) || "??"
            } Wh (est. cap.)`,
          });
        } else {
          // display acceleration, in Wh per hour^2, ie W/h, rounded to aribtrary stdev of 0.5 W/h
          const roundedVel = floorVal((accelSlow || 0) * 3600, 0.5);
          // display acceleration, in % per hour
          const dispAccel = (valFast && roundedVel / valFast) || 0;
          let dir = "→";
          if (dispAccel < -1) dir = "↓";
          else if (dispAccel < -0.5) dir = "↘";
          if (dispAccel > 1) dir = "↑";
          else if (dispAccel > 0.5) dir = "↗";
          node.status({
            fill: props.cusum < 0.05 ? "green" : "yellow",
            shape: "ring",
            text: `${dir} ${valFast.toFixed(2)} W - total ${nrg.toFixed(1)} Wh`,
          });
        }

        nodeSend([
          { payload: valFast, topic: "value" }, // this is rate (Wh / h)
          { payload: valFast && (3600 * accelSlow) / valFast, topic: "rate" }, // this is accel %(Wh / h^2)
          null,
        ]);

        props.timeout = setTimeout(() => {
          // in timeout loop => predicts, evaluate cusum, pushResult
          const nextNow = performance.now();
          const ts1 = props.fastFilter.predict(nextNow);
          const ts2 = props.slowFilter.predict(nextNow);
          mainLoop(nextNow, ts1);
        }, timeout);
      };

      if (pv !== null && !isNaN(pv) && isFinite(pv)) {
        // in value => predicts, check for reset (filter, cusum, not energy?), or correct, evaluate cusum, pushResult

        const now = performance.now();

        const ts1 = props.fastFilter.predict(now);
        const ts2 = props.slowFilter.predict(now);

        const [predVal] = props.fastFilter.mean();

        // check for reset condition
        if (
          predVal === null ||
          (predVal + pv && !(predVal * pv)) || // start or stop
          Math.abs((pv - predVal) / pv) > 0.1 // 10% step triggers a reset
        ) {
          // reset
          props.slowFilter.resetCovariance(pv);
          props.fastFilter.resetCovariance(pv);
          if (predVal === 0 && pv) {
            props.before = 0;
            props.cusum = 0;
          }
        } else {
          props.slowFilter.correct(pv, ts1);
          props.fastFilter.correct(pv, ts2);
        }
        mainLoop(now, ts1, 5500);
      } else {
        node.status({ fill: "red", shape: "dot", text: "Bad input value" });
      }
      if (done) done();
    });

    node.on("close", () => {
      if (props.timeout) clearTimeout(props.timeout);
      props = initProps();
    });
  }

  RED.nodes.registerType("smartcharge-monitor", Monitor);
};

module.exports = nodeInit;
