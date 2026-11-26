const { IMMFilter } = require("./imm-filter");

/** @typedef {import("node-red").NodeMessage} NodeMessage */

const stdK = 0.77e-7;

/** cusum mini k value / minimum active decay rate (in hours^-1) */
const w = 1 / 6.66;

/** one-sided Wald z-score threshold for triggering OFF (~99.87% confidence k > w) */
const waldTriggerZ = 3;

/** one-sided Wald z-score threshold for highlighting active decay in node status */
const waldYellowZ = 1.5;

/** max elapsed ms since last measurement to extrapolate in scheduleUpdate */
const maxPredictGapMs = 30000;

/**
 * Parse a numeric Node-RED config field safely (handles strings and 0).
 * @param {number | string | undefined} raw
 * @param {number} fallback
 * @param {object} [bounds]
 * @param {number} [bounds.minExclusive]
 * @param {number} [bounds.minInclusive]
 * @param {number} [bounds.maxExclusive]
 * @param {number} [bounds.maxInclusive]
 * @return {number}
 */
function parseConfigNumber(raw, fallback, bounds = {}) {
  const isNumericInput =
    typeof raw === "number" ||
    (typeof raw === "string" && raw.trim() !== "");
  if (!isNumericInput) return fallback;
  const n = Number(raw);
  if (!Number.isFinite(n)) return fallback;
  if (bounds.minExclusive !== undefined && n <= bounds.minExclusive) return fallback;
  if (bounds.minInclusive !== undefined && n < bounds.minInclusive) return fallback;
  if (bounds.maxExclusive !== undefined && n >= bounds.maxExclusive) return fallback;
  if (bounds.maxInclusive !== undefined && n > bounds.maxInclusive) return fallback;
  return n;
}

/** @type {import("node-red").NodeInitializer} */
const nodeInit = (RED) => {
  /**
   * @this {import("node-red").Node}
   * @param {import("./types").MonitorConfig} config
   */
  function Monitor(config) {
    // DEBUG
    // const fdata = `/tmp/data-${new Date().toISOString()}.csv`;
    // fs.appendFileSync(fdata, "timestamp,value\n");

    RED.nodes.createNode(this, config);
    const node = this;

    /** @type {number} peak charge% */
    const peakCharge = parseConfigNumber(config.peakCharge, 80, {
      minInclusive: 0,
      maxExclusive: 100,
    });

    /** @type {number} target charge% */
    const cutoffFallback = Math.max(85, Math.min(100, peakCharge + 5));
    const maxCharge = parseConfigNumber(config.cutoff, cutoffFallback, {
      minExclusive: peakCharge,
      maxInclusive: 100,
    });

    /** @type {number} charger efficiency */
    const chargerEfficiency = parseConfigNumber(config.chargerEfficiency, 0.72, {
      minExclusive: 0,
      maxInclusive: 1,
    });

    /** @type {number} power standard deviation */
    const pwrStdev = parseConfigNumber(config.pwrStdev, 0.15, {
      minExclusive: 0,
    });

    /** @type {import("./types").Props} */
    let props;

    const initProps = (/** @type {number} */ now) => {
      /** @type {import("./types").Props} */
      const p = {
        imm: new IMMFilter({ stdK, pwrStdev }),
        lastPush: now,
        localPeak: 0,
        inDecay: false,
        energy: 0,
        maxPwr: 0,
        timeout: null,
        finishTimeout: null,
        finishing: false,
        offSent: false,
        startTime: now,
      };
      p.imm.init(0, now);
      return p;
    };

    props = initProps(Date.now());
    node.log("Props initialized");

    const getFused = () => props.imm.mean();

    const evalTrigger = (
      /** @type {(msg: NodeMessage | Array<NodeMessage | NodeMessage[] | null>) => void} */ nodeSend
    ) => {
      if (props.finishing || props.offSent) return false;

      const threshold = (maxCharge - peakCharge) / (100 - peakCharge); // 5 to compensate for actual end of charge @90% of saturation (TBC)
      const [val, k] = getFused();
      if (val === null || k === null) return false;

      const kZ = props.imm.kZScore(w);
      const powerRatio = 1 - 0.9 * threshold; // assumes 100% at 10% of max power - FIXME constant

      if (
        kZ >= waldTriggerZ &&
        k * 3600 > w && // must still be actively decaying at trigger time
        props.maxPwr &&
        val <= props.maxPwr * powerRatio &&
        (!props.localPeak || val <= props.localPeak * powerRatio)
      ) {
        node.log("Triggered");
        node.log(
          `Scheduling one time OFF, ${val.toFixed(1)} / ${props.maxPwr.toFixed(
            1
          )} W, k=${(k * 3600).toFixed(2)}`
        );
        props.finishing = true;
        // wrap in timeout to avoid simultaneous read / write on some devices (Meross)
        props.finishTimeout = setTimeout(() => {
          props.finishTimeout = null;
          if (props.finishing) {
            node.log("Sending OFF");
            nodeSend([
              { payload: 0 }, // optimistic zero
              { payload: 0 },
              { payload: false }, // OFF payload
            ]);
            props.finishing = false;
            props.offSent = true;
            node.status({
              fill: "blue",
              shape: "dot",
              text: `OFF sent - total ${(props.energy / 3600).toFixed(1)} Wh`,
            });
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
      if (val === null || k === null) return;

      const dt = Math.max(0, (now - props.lastPush) / 1e3);
      props.energy += Math.max(0, val) * chargerEfficiency * dt;
      props.lastPush = Math.max(now, props.lastPush);

      /** in Wh */
      const nrg = props.energy / 3600;

      const accel = k * 3600 * (0 - val) || 0;

      const kZ = props.imm.kZScore(w);
      const hasDecay = kZ >= waldYellowZ && k * 3600 > w;
      // Retain peak power observed while not in decay
      if (k * 3600 < w) props.maxPwr = Math.max(props.maxPwr, val);
      // Track segment plateau peak; re-anchor if returning from an aborted decay excursion
      if (kZ <= 0) {
        props.localPeak = props.inDecay ? val : Math.max(props.localPeak, val);
        props.inDecay = false;
      } else {
        props.inDecay = true;
      }

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
        if (props.offSent) {
          node.status({
            fill: "blue",
            shape: "dot",
            text: `${dir} ${val.toFixed(2)} W - OFF sent (${nrg.toFixed(1)} Wh)`,
          });
        } else {
          node.status({
            fill: hasDecay ? "yellow" : "green",
            shape: "ring",
            text: `${dir} ${val.toFixed(2)} W - total ${nrg.toFixed(1)} Wh`,
          });
        }
      }

      nodeSend([
        { payload: val, topic: "value" }, // this is rate (W)
        {
          payload: accel, // rate of change of power (W / h)
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
        props.timeout = null;
        const now = Date.now();
        if (now - props.imm.correctTS >= maxPredictGapMs) {
          return;
        }
        props.imm.predict(now);
        const [predVal] = getFused();
        if (predVal !== null && predVal <= 0) {
          return;
        }
        pushOut(now, nodeSend);
        if (!evalTrigger(nodeSend) && !props.offSent) scheduleUpdate(nodeSend);
      }, timeout);
    };

    node.on("input", (msg, send, done) => {
      const rawPayload = msg.payload;
      const isNumericInput =
        typeof rawPayload === "number" ||
        (typeof rawPayload === "string" && rawPayload.trim() !== "");
      const pv = isNumericInput ? Number(rawPayload) : NaN;
      const isValidPv = Number.isFinite(pv) && pv >= 0;

      if (props.finishing) {
        if (isValidPv && pv === 0) {
          // Real Stop arrived during the 2 s delay; cancel OFF timer and process Stop
          if (props.finishTimeout) {
            clearTimeout(props.finishTimeout);
            props.finishTimeout = null;
          }
          props.finishing = false;
        } else {
          if (done) done();
          return;
        }
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

      if (isValidPv) {
        // in value => predicts, check for reset, or correct, pushResult and evaluate trigger
        const now = Date.now();

        const reInit = (accPwr = 0) => {
          if (accPwr > 0 && now > props.lastPush) {
            props.energy +=
              accPwr * chargerEfficiency * ((now - props.lastPush) / 1e3);
          }
          props.imm.init(pv, now);
          props.lastPush = now;
          props.localPeak = pv;
          props.inDecay = false;
          pushOut(now, nodeSend);
          if (pv !== 0 && !props.offSent) scheduleUpdate(nodeSend);
          if (done) done();
        };

        const [prevVal] = props.imm.mean();

        // Check for reset conditions

        // first detect start or stop
        if (prevVal === null || (prevVal + pv && !(prevVal * pv))) {
          if (pv) {
            node.log("Start detected");
            props.energy = 0;
            props.maxPwr = 0;
            props.offSent = false;
            props.startTime = now;
            reInit(0);
          } else {
            node.log("Stop detected");
            props.maxPwr = 0;
            props.offSent = false;
            reInit(prevVal ?? 0);
          }
          return;
        }

        props.imm.step(pv, now);

        const { nis, innovation } = props.imm.fastInnovation();
        const mahalanobis = Math.max(0, nis) ** 0.5;
        const absInnovation = Math.abs(innovation);
        if (mahalanobis > 3 && absInnovation >= 0.2 * Math.abs(pv)) {
          // reset !
          reInit(prevVal ?? 0);
          return;
        }

        pushOut(now, nodeSend);
        const triggered = pv !== 0 && evalTrigger(nodeSend);
        if (pv !== 0 && !triggered && !props.offSent) scheduleUpdate(nodeSend); // TODO: evalTrigger should call scheduleUpdate ?
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
      if (props.finishTimeout) {
        clearTimeout(props.finishTimeout);
        props.finishTimeout = null;
      }
      props = initProps(Date.now());
    });
  }

  RED.nodes.registerType("smartcharge-monitor", Monitor);
};

module.exports = nodeInit;
