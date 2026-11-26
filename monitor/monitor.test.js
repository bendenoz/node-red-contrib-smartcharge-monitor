const fs = require("fs");
const path = require("path");
const { parse: csvParse } = require("csv-parse/sync");

const nodeInit = require("./monitor");

const startTime = new Date(2023, 0, 1).getTime();

describe("Monitor Node", () => {
  let RED;
  let node;
  let config;
  let send;
  let done;

  beforeEach(() => {
    jest.useFakeTimers();
    jest.setSystemTime(new Date(startTime));

    RED = {
      nodes: {
        createNode: jest.fn(),
        registerType: jest.fn(),
      },
    };
    node = {
      log: jest.fn(),
      status: jest.fn(),
      send: jest.fn(),
      on: jest.fn((event, callback) => {
        if (event === "input") {
          node.inputCallback = callback;
        } else if (event === "close") {
          node.closeCallback = callback;
        }
      }),
    };
    config = { cutoff: 85 };
    send = jest.fn();
    done = jest.fn();
  });

  afterEach(() => {
    jest.useRealTimers();
    jest.restoreAllMocks();
  });

  it("should initialize properties correctly", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    expect(node.log).toHaveBeenCalledWith("Props initialized");
  });

  it("should handle input messages correctly", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    const msg = { payload: 50 };
    node.inputCallback(msg, send, done);

    expect(node.log).toHaveBeenCalledWith("Start detected");
    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
  });

  it("should reject non-numeric, empty, boolean, null, and negative payloads", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    for (const bad of ["bad value", "", "   ", null, false, [], -1]) {
      node.status.mockClear();
      done.mockClear();
      node.inputCallback({ payload: bad }, send, done);
      expect(node.status).toHaveBeenCalledWith({
        fill: "red",
        shape: "dot",
        text: "Bad input value",
      });
      expect(done).toHaveBeenCalled();
    }
    expect(node.log).not.toHaveBeenCalledWith("Stop detected");
  });

  it("should clear pending scheduleUpdate timeout on close", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    node.inputCallback({ payload: 50 }, send, done);
    const callsBeforeClose = send.mock.calls.length;

    node.closeCallback();
    jest.advanceTimersByTime(15000);

    expect(send.mock.calls.length).toBe(callsBeforeClose);
  });

  it("should not schedule predict-only timers while idle at 0 W", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    node.inputCallback({ payload: 0 }, send, done);
    const callsAfterZero = send.mock.calls.length;

    jest.advanceTimersByTime(20000);
    expect(send.mock.calls.length).toBe(callsAfterZero);
  });

  it("should stop predict-only extrapolation after 30 s without measurements", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    let t = 0;
    for (let i = 0; i < 10; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 50 }, send, done);
    }
    // Small dip (< 20% so no Mahalanobis reset, leaving k > 0), then telemetry goes silent
    t += 20000;
    jest.setSystemTime(new Date(startTime + t));
    node.inputCallback({ payload: 44 }, send, done);
    const callsAtSilence = send.mock.calls.length;

    jest.advanceTimersByTime(600000);
    // At most ~5 ticks within the 30 s horizon, and no false OFF
    expect(send.mock.calls.length - callsAtSilence).toBeLessThanOrEqual(6);
    const offCalls = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCalls).toHaveLength(0);

    node.closeCallback();
  });

  it("should handle Stop (0 W) during the 2 s finishing window without wedging next session", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    let t = 0;
    for (let i = 0; i < 10; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 50 }, send, done);
    }
    const kTrue = 2.5 / 3600;
    let triggered = false;
    for (let i = 1; i <= 30 && !triggered; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 50 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      triggered = node.log.mock.calls.some(([m]) => m === "Triggered");
    }
    expect(triggered).toBe(true);

    // Plug drops to 0 W 500 ms into the 2 s delay
    t += 500;
    jest.setSystemTime(new Date(startTime + t));
    node.inputCallback({ payload: 0 }, send, done);
    expect(node.log).toHaveBeenCalledWith("Stop detected");

    // Advance past the 2 s window -> canceled finishTimeout must not emit OFF
    jest.advanceTimersByTime(3000);
    let offCalls = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCalls).toHaveLength(0);

    // Next session should work normally and still emit OFF
    for (let i = 0; i < 12; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 30 }, send, done);
    }
    for (let i = 1; i <= 25; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 30 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      jest.advanceTimersByTime(2500);
    }
    offCalls = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCalls).toHaveLength(1);

    node.closeCallback();
  });

  it("should stop predict-only ticks when extrapolated power reaches <= 0", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    let t = 0;
    for (let i = 0; i < 6; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 50 }, send, done);
    }
    // Rapidly steep decay (< 20% jump) so k is high and linear extrapolation crosses 0 within < 30s
    t += 20000;
    jest.setSystemTime(new Date(startTime + t));
    node.inputCallback({ payload: 41 }, send, done);

    jest.advanceTimersByTime(30000);
    const emittedPwrs = send.mock.calls.map(([msgs]) => msgs[0] && msgs[0].payload);
    expect(emittedPwrs.every((p) => p === undefined || p > 0)).toBe(true);

    node.closeCallback();
  });

  it("should safely parse string, whitespace, boolean, and cross-field config values", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    // peakCharge=90 with invalid/low cutoff=85 should fall back cutoff to 95 (> 90), not 85
    Monitor.call(node, {
      cutoff: "85",
      peakCharge: "90",
      chargerEfficiency: "   ",
      pwrStdev: /** @type {any} */ (false),
    });

    let t = 0;
    for (let i = 0; i < 12; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 50 }, send, done);
    }
    // Drop 6% (to 47 W) with decay -> should NOT trigger since cutoff fell back to 95% (~55% of max)
    const kTrue = 2.0 / 3600;
    for (let i = 1; i <= 4; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 50 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      jest.advanceTimersByTime(2500);
    }
    const offCalls = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCalls).toHaveLength(0);

    node.closeCallback();
  });

  it("should continue predict-only ticks via scheduleUpdate after Start and Mahalanobis reset", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    // Start at 70 W
    node.inputCallback({ payload: 70 }, send, done);
    expect(send).toHaveBeenCalledTimes(1);

    // Predict-only tick at +5.5s
    jest.advanceTimersByTime(5500);
    expect(send).toHaveBeenCalledTimes(2);

    // Small jump (< 20% of pv): should NOT reset, smooths gently
    jest.setSystemTime(new Date(startTime + 20000));
    node.inputCallback({ payload: 72.5 }, send, done);
    const afterSmallJump = send.mock.calls[send.mock.calls.length - 1][0][0].payload;
    expect(afterSmallJump).toBeLessThan(72.5);

    // Large step (>= 20% of pv and > 3 sigma): triggers reInit (exact payload) and keeps scheduling
    jest.setSystemTime(new Date(startTime + 40000));
    node.inputCallback({ payload: 100 }, send, done);
    const afterReset = send.mock.calls[send.mock.calls.length - 1][0][0].payload;
    expect(afterReset).toBe(100);

    const callsBeforeTimer = send.mock.calls.length;
    jest.advanceTimersByTime(5500);
    expect(send.mock.calls.length).toBe(callsBeforeTimer + 1);

    node.closeCallback();
  });

  it("should latch OFF until Stop detected and reset maxPwr between sessions", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    // Session 1: 100 W steady plateau, then decay down past cutoff
    let t = 0;
    for (let i = 0; i < 15; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 100 }, send, done);
    }
    // Slight dip while still flat should not shrink maxPwr (e.g. 99 W)
    t += 20000;
    jest.setSystemTime(new Date(startTime + t));
    node.inputCallback({ payload: 99 }, send, done);

    const kTrue = 2.0 / 3600;
    for (let i = 1; i <= 25; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 100 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      jest.advanceTimersByTime(2500);
    }

    const offCalls = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCalls).toHaveLength(1);
    // Peak maxPwr was logged as 100.0 W (not shrunk to 99.0 W)
    const schedLog = node.log.mock.calls.find(
      ([msg]) => typeof msg === "string" && msg.startsWith("Scheduling one time OFF")
    );
    expect(schedLog[0]).toContain("/ 100.0 W");

    // Continue sending decaying power -> should NOT emit a second OFF
    for (let i = 26; i <= 35; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 100 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      jest.advanceTimersByTime(2500);
    }
    const offCallsAfter = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCallsAfter).toHaveLength(1);

    // Stop detected (0 W), then new 10 W session should reset maxPwr and allow a fresh OFF
    t += 20000;
    jest.setSystemTime(new Date(startTime + t));
    node.inputCallback({ payload: 0 }, send, done);

    for (let i = 0; i < 15; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 10 }, send, done);
    }
    for (let i = 1; i <= 25; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 10 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      jest.advanceTimersByTime(2500);
    }
    const offCallsSession2 = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCallsSession2).toHaveLength(2);
    const schedLogs = node.log.mock.calls.filter(
      ([msg]) => typeof msg === "string" && msg.startsWith("Scheduling one time OFF")
    );
    expect(schedLogs[1][0]).toContain("/ 10.0 W");

    node.closeCallback();
  });

  it("should clear pending 2s finishTimeout on close without sending OFF", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    let t = 0;
    for (let i = 0; i < 10; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 50 }, send, done);
    }

    const kTrue = 2.5 / 3600;
    let triggered = false;
    for (let i = 1; i <= 30 && !triggered; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 50 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      triggered = node.log.mock.calls.some(([m]) => m === "Triggered");
    }
    expect(triggered).toBe(true);

    // Close before the 2s finishTimeout expires
    node.closeCallback();
    jest.advanceTimersByTime(5000);

    const offCalls = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCalls).toHaveLength(0);
  });

  it("should keep yellow active-decay status across a single noisy sample", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    let t = 0;
    for (let i = 0; i < 15; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 50 }, send, done);
    }

    // Decay until yellow status (waldYellowZ) appears
    const kTrue = 2.0 / 3600;
    let sawYellow = false;
    for (let i = 1; i <= 10; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 50 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      const lastStatus =
        node.status.mock.calls[node.status.mock.calls.length - 1][0];
      if (lastStatus.fill === "yellow") {
        sawYellow = true;
        break;
      }
    }
    expect(sawYellow).toBe(true);

    // A single flat/noisy sample should not clear active-decay status
    t += 20000;
    jest.setSystemTime(new Date(startTime + t));
    node.inputCallback({ payload: 49 }, send, done);
    const statusAfterNoise =
      node.status.mock.calls[node.status.mock.calls.length - 1][0];
    expect(statusAfterNoise.fill).toBe("yellow");

    node.closeCallback();
  });

  const replayFixture = (fixtureFile, monitorConfig) => {
    const dataPath = path.join(__dirname, "..", "fixtures", fixtureFile);
    const csvData = fs.readFileSync(dataPath, "utf8");
    const inputData = csvParse(csvData, {
      columns: true,
      skip_empty_lines: true,
      cast: true,
    });

    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, monitorConfig);

    const logs = [];
    node.log = jest.fn((...args) => {
      logs.push({ timeMin: (Date.now() - startTime) / 60000, args });
    });

    const offEvents = [];
    const trackedSend = jest.fn((msgs) => {
      if (Array.isArray(msgs) && msgs[2] && msgs[2].payload === false) {
        offEvents.push({ timeMin: (Date.now() - startTime) / 60000 });
      }
      send(msgs);
    });

    inputData.forEach((data, idx) => {
      const timestamp = parseFloat(data.timestamp);
      const value = parseFloat(data.value);

      jest.setSystemTime(new Date(startTime + timestamp));
      node.inputCallback({ payload: value }, trackedSend, done);

      const nextTimestamp =
        idx + 1 < inputData.length
          ? parseFloat(inputData[idx + 1].timestamp)
          : timestamp + 5500;
      const gapMs = Math.max(0, Math.floor(nextTimestamp - timestamp) - 1);
      if (gapMs > 0) {
        jest.advanceTimersByTime(gapMs);
      }
    });

    node.closeCallback();

    const schedLog = logs.find(
      (l) => typeof l.args[0] === "string" && l.args[0].startsWith("Scheduling one time OFF")
    );
    let triggerInfo = null;
    if (schedLog) {
      const m = schedLog.args[0].match(/OFF,\s*([\d.]+)\s*\/\s*([\d.]+)\s*W,\s*k=([\d.-]+)/);
      if (m) {
        triggerInfo = {
          powerW: parseFloat(m[1]),
          maxPwrW: parseFloat(m[2]),
          kPerHour: parseFloat(m[3]),
        };
      }
    }

    return { logs, offEvents, triggerInfo };
  };

  // Ninebot trace ends at ~86.2% of plateau power (manually unplugged before 85% threshold),
  // so cutoff: 82 (~91% plateau threshold) validates CV detection before unplug.
  it("should process ninebot fixture data correctly", () => {
    const { logs, offEvents, triggerInfo } = replayFixture("data-ninebot.csv", {
      cutoff: 82,
    });
    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
    expect(logs.some((l) => l.args[0] === "Start detected")).toBe(true);
    expect(logs.some((l) => l.args[0] === "Stop detected")).toBe(true);
    expect(offEvents).toHaveLength(1);
    expect(offEvents[0].timeMin).toBeGreaterThan(95);
    expect(offEvents[0].timeMin).toBeLessThan(101);
    expect(triggerInfo).not.toBeNull();
    expect(triggerInfo.powerW).toBeLessThan(triggerInfo.maxPwrW);
  });

  it("should process powerbank fixture data correctly", () => {
    const { logs, offEvents, triggerInfo } = replayFixture(
      "data-noisy-powerbank.csv",
      { cutoff: 82 }
    );
    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
    expect(logs.some((l) => l.args[0] === "Start detected")).toBe(true);
    expect(logs.some((l) => l.args[0] === "Stop detected")).toBe(true);
    expect(offEvents).toHaveLength(1);
    expect(offEvents[0].timeMin).toBeGreaterThan(310);
    expect(offEvents[0].timeMin).toBeLessThan(330);
    expect(triggerInfo).not.toBeNull();
    expect(triggerInfo.powerW).toBeLessThan(triggerInfo.maxPwrW);
  });

  it("should process powerbank fixture with default cutoff (85%)", () => {
    const { logs, offEvents, triggerInfo } = replayFixture(
      "data-noisy-powerbank.csv",
      {}
    );
    expect(logs.some((l) => l.args[0] === "Start detected")).toBe(true);
    expect(logs.some((l) => l.args[0] === "Stop detected")).toBe(true);
    expect(offEvents).toHaveLength(1);
    expect(offEvents[0].timeMin).toBeGreaterThan(320);
    expect(offEvents[0].timeMin).toBeLessThan(340);
    expect(triggerInfo).not.toBeNull();
    expect(triggerInfo.powerW).toBeLessThan(triggerInfo.maxPwrW * 0.8);
  });

  // Watches draw active OS/display loads alongside battery charging; cutoff: 83 matches historical test target.
  it("should process applewatch fixture data correctly", () => {
    const { logs, offEvents, triggerInfo } = replayFixture(
      "data-apple-watch.csv",
      { cutoff: 83 }
    );
    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
    expect(logs.some((l) => l.args[0] === "Start detected")).toBe(true);
    expect(offEvents).toHaveLength(1);
    expect(offEvents[0].timeMin).toBeGreaterThan(20);
    expect(offEvents[0].timeMin).toBeLessThan(112);
    expect(triggerInfo).not.toBeNull();
    expect(triggerInfo.powerW).toBeLessThan(triggerInfo.maxPwrW);
  });

  it("should process samsungwatch fixture data correctly", () => {
    const { logs, offEvents, triggerInfo } = replayFixture(
      "data-samsung-watch.csv",
      { cutoff: 83 }
    );
    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
    expect(logs.some((l) => l.args[0] === "Start detected")).toBe(true);
    expect(offEvents).toHaveLength(1);
    expect(offEvents[0].timeMin).toBeGreaterThan(60);
    expect(offEvents[0].timeMin).toBeLessThan(72);
    expect(triggerInfo).not.toBeNull();
    expect(triggerInfo.powerW).toBeLessThan(triggerInfo.maxPwrW);
  });

  it("should re-anchor localPeak on multi-stage USB-PD step-downs and show OFF sent status", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 85 });

    let t = 0;
    // Stage 1: 65 W for 15 minutes
    for (let i = 0; i < 45; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 65 }, send, done);
    }
    // Stage 2: -50% PD step-down to 32 W flat for 15 minutes (below 65 * 0.775)
    for (let i = 0; i < 45; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 32 }, send, done);
    }
    // Stage 3: genuine CV taper from 32 W -> should wait for ~24.8 W (32 * 0.775), not fire at 30 W
    const kTrue = 1.5 / 3600;
    for (let i = 1; i <= 40; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback(
        { payload: 32 * Math.exp(-kTrue * i * 20) },
        send,
        done
      );
      jest.advanceTimersByTime(2500);
    }

    const offCalls = send.mock.calls.filter(
      ([msgs]) => msgs[2] && msgs[2].payload === false
    );
    expect(offCalls).toHaveLength(1);

    const schedLog = node.log.mock.calls.find(
      ([msg]) => typeof msg === "string" && msg.startsWith("Scheduling one time OFF")
    );
    expect(schedLog).toBeDefined();
    const match = schedLog[0].match(/OFF,\s*([\d.]+)\s*\/\s*([\d.]+)\s*W/);
    expect(match).not.toBeNull();
    expect(parseFloat(match[1])).toBeLessThanOrEqual(25.5);

    const statusTexts = node.status.mock.calls.map(([s]) => s.text);
    expect(statusTexts.some((txt) => txt.includes("OFF sent"))).toBe(true);

    node.closeCallback();
  });

  it("should preserve interval energy across Mahalanobis step resets", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 85, chargerEfficiency: 1 });

    // Steady 60 W at 20 s cadence for 580 s, then step jump to 100 W at 600 s (10 min = 10.0 Wh)
    for (let i = 0; i < 30; i++) {
      jest.setSystemTime(new Date(startTime + i * 20000));
      node.inputCallback({ payload: 60 }, send, done);
    }
    jest.setSystemTime(new Date(startTime + 600000));
    node.inputCallback({ payload: 100 }, send, done);

    // Must preserve the 580s -> 600s slice (10.0 Wh total, not 9.7 Wh) and trigger reInit (100 W exact)
    const lastMsgs = send.mock.calls[send.mock.calls.length - 1][0];
    expect(lastMsgs[0].payload).toBe(100);
    const lastStatus =
      node.status.mock.calls[node.status.mock.calls.length - 1][0];
    expect(lastStatus.text).toContain("total 10.0 Wh");

    node.closeCallback();
  });
});
