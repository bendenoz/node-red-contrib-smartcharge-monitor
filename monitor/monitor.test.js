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
  let inputData;
  let originalPerformance;

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

    // Save the original performance
    originalPerformance = global.performance;
  });

  afterEach(() => {
    jest.useRealTimers();
    jest.restoreAllMocks();

    // Restore the original performance
    global.performance = originalPerformance;
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

  it("should handle bad input values", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    const msg = { payload: "bad value" };
    node.inputCallback(msg, send, done);

    expect(node.status).toHaveBeenCalledWith({
      fill: "red",
      shape: "dot",
      text: "Bad input value",
    });
    expect(done).toHaveBeenCalled();
  });

  it("should clear timeout on close", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, config);

    node.closeCallback();

    expect(node.log).toHaveBeenCalledWith("Props initialized");
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

  it("should accumulate CUSUM without wiping on a single noisy sample", () => {
    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    let t = 0;
    for (let i = 0; i < 15; i++) {
      t += 20000;
      jest.setSystemTime(new Date(startTime + t));
      node.inputCallback({ payload: 50 }, send, done);
    }

    // Decay until yellow status (hasDecay: triggerCusum > w) appears
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

    // A single flat/noisy sample should not hard-reset triggerCusum to 0
    t += 20000;
    jest.setSystemTime(new Date(startTime + t));
    node.inputCallback({ payload: 47 }, send, done);
    const statusAfterNoise =
      node.status.mock.calls[node.status.mock.calls.length - 1][0];
    expect(statusAfterNoise.fill).toBe("yellow");

    node.closeCallback();
  });

  const replayFixture = (fixtureFile, monitorConfig) => {
    const dataPath = path.join(__dirname, "..", "fixtures", fixtureFile);
    const csvData = fs.readFileSync(dataPath, "utf8");
    inputData = csvParse(csvData, {
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
    expect(offEvents[0].timeMin).toBeGreaterThan(20);
    expect(offEvents[0].timeMin).toBeLessThan(96);
    expect(triggerInfo).not.toBeNull();
    expect(triggerInfo.powerW).toBeLessThan(triggerInfo.maxPwrW);
  });
});
