const fs = require("fs");
const path = require("path");
const { parse: csvParse } = require("csv-parse/sync");

const { KalmanFilter } = require("./kalman-filter");
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

  it("should process ninebot fixture data correctly", () => {
    const dataPath = path.join(__dirname, "..", "fixtures", "data-ninebot.csv");
    const csvData = fs.readFileSync(dataPath, "utf8");
    inputData = csvParse(csvData, {
      columns: true,
      skip_empty_lines: true,
      cast: true,
    });

    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    node.log = jest.fn((...args) => [args, Date.now()]);

    inputData.forEach((data) => {
      const timestamp = parseFloat(data.timestamp);
      const value = parseFloat(data.value);

      jest.setSystemTime(new Date(startTime + timestamp));

      const msg = { payload: value };
      node.inputCallback(msg, send, done);

      jest.advanceTimersByTime(5500);
      jest.advanceTimersByTime(5500);
      jest.advanceTimersByTime(5500);
    });

    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
    expect(node.log).toHaveBeenCalled();

    // Output the log calls to the console with system time of the actual call
    console.log(
      node.log.mock.results.map((result) => [
        (result.value[1] - startTime) / 1000,
        (result.value[1] - startTime) / 1000 / 60,
        result.value[0],
      ])
    );

    node.closeCallback();
  });

  it("should process powerbank fixture data correctly", () => {
    const dataPath = path.join(
      __dirname,
      "..",
      "fixtures",
      "data-noisy-powerbank.csv"
    );
    const csvData = fs.readFileSync(dataPath, "utf8");
    inputData = csvParse(csvData, {
      columns: true,
      skip_empty_lines: true,
      cast: true,
    });

    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 82 });

    node.log = jest.fn((...args) => [args, Date.now()]);

    inputData.forEach((data) => {
      const timestamp = parseFloat(data.timestamp);
      const value = parseFloat(data.value);

      jest.setSystemTime(new Date(startTime + timestamp));

      const msg = { payload: value };
      node.inputCallback(msg, send, done);

      jest.advanceTimersByTime(5500);
      jest.advanceTimersByTime(5500);
      jest.advanceTimersByTime(5500);
    });

    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
    expect(node.log).toHaveBeenCalled();

    // Output the log calls to the console with system time of the actual call
    console.log(
      node.log.mock.results.map((result) => [
        (result.value[1] - startTime) / 1000,
        (result.value[1] - startTime) / 1000 / 60,
        result.value[0],
      ])
    );

    node.closeCallback();
  });

  it("should process applewatch fixture data correctly", () => {
    const dataPath = path.join(
      __dirname,
      "..",
      "fixtures",
      "data-apple-watch.csv"
    );
    const csvData = fs.readFileSync(dataPath, "utf8");
    inputData = csvParse(csvData, {
      columns: true,
      skip_empty_lines: true,
      cast: true,
    });

    nodeInit(RED);
    const Monitor = RED.nodes.registerType.mock.calls[0][1];
    Monitor.call(node, { cutoff: 83 });

    node.log = jest.fn((...args) => [args, Date.now()]);

    inputData.forEach((data) => {
      const timestamp = parseFloat(data.timestamp);
      const value = parseFloat(data.value);

      jest.setSystemTime(new Date(startTime + timestamp));

      const msg = { payload: value };
      node.inputCallback(msg, send, done);

      jest.advanceTimersByTime(5500);
      jest.advanceTimersByTime(5500);
      jest.advanceTimersByTime(5500);
    });

    expect(send).toHaveBeenCalled();
    expect(done).toHaveBeenCalled();
    expect(node.log).toHaveBeenCalled();

    // Output the log calls to the console with system time of the actual call
    console.log(
      node.log.mock.results.map((result) => [
        (result.value[1] - startTime) / 1000,
        (result.value[1] - startTime) / 1000 / 60,
        result.value[0],
      ])
    );

    node.closeCallback();
  });
});
