const fs = require("fs");
const path = require("path");
const { parse: csvParse } = require("csv-parse/sync");
const { matMul, transpose } = require("simple-linalg");

const { KalmanFilter } = require("./kalman-filter");

describe("Kalman tests", () => {
  test("Power Bank", () => {
    const dataPath = path.join(
      __dirname,
      "..",
      "fixtures",
      "data-noisy-powerbank.csv"
    );
    const csvData = fs.readFileSync(dataPath, "utf8");
    const inputData = csvParse(csvData, {
      columns: true,
      skip_empty_lines: true,
      cast: true,
    });

    // Mirror monitor.js: create a slow and a fast filter and run the fixture through them
    const kfSlow = new KalmanFilter(0.77e-7, 0.15);
    const kfFast = new KalmanFilter(50 * 0.77e-7, 0.15);

    // Initialize with first sample timestamp as base time
    const firstTs = Math.floor(parseFloat(inputData[0].timestamp));
    kfSlow.init(parseFloat(inputData[0].value), firstTs);
    kfFast.init(parseFloat(inputData[0].value), firstTs);

    let lastTS = firstTs;
    const results = [];
    let mu = [0, 1]; // fast at first
    const p11 = 0.995;
    const p22 = 0.95;
    const Pi = [
      [p11, 1 - p11], // from slow -> [slow, fast]
      [1 - p22, p22], // from fast -> [slow, fast]
    ];

    inputData.forEach((row) => {
      const ts = Math.floor(parseFloat(row.timestamp));
      const val = parseFloat(row.value);

      const timestep = (ts - kfSlow.stateTS) / 1000;
      kfSlow.predict(ts);
      kfSlow.correct(val, ts);

      kfFast.predict(ts);
      kfFast.correct(val, ts);

      const logL = /** @type {number[]} */ ([
        kfSlow.logL ?? Number.NEGATIVE_INFINITY,
        kfFast.logL ?? Number.NEGATIVE_INFINITY,
      ]);

      const muPrior = matMul(transpose(Pi), [[mu[0]], [mu[1]]]).map(
        (v) => v[0]
      );
      // normalize and update mu (numerically stable, guard against under/overflow)
      const maxLogL = Math.max(...logL);
      let weights = muPrior.map((m, i) => m * Math.exp(logL[i] - maxLogL));
      let muSum = weights.reduce((a, b) => a + b, 0);
      if (!isFinite(muSum) || muSum === 0) {
        // fallback to uniform prior if weights collapsed
        weights = [0.5, 0.5];
        muSum = 1;
      }
      mu = weights.map((w) => w / muSum);

      const [meanSlow, kSlow] = /** @type {number[]} */ (
        kfSlow.mean() || [0, 0]
      );
      const [meanFast, kFast] = /** @type {number[]} */ (
        kfFast.mean() || [0, 0]
      );
      const fused = mu[0] * meanSlow + mu[1] * meanFast;
      const fusedK = mu[0] * kSlow + mu[1] * kFast;

      results.push([
        ts / 1000,
        val,
        fused,
        3600 * fusedK,
        mu[0],
        meanSlow,
        meanFast,
      ]);

      lastTS = ts;
    });

    // console.log(results);

    const outDir = path.join(__dirname, "..", "out");
    fs.mkdirSync(outDir, { recursive: true });
    const outPath = path.join(outDir, "powerbank.csv");
    const header = "timestamp,value,fused,k,mu,slow,fast";
    const csvLines = results.map(
      (row) =>
        `${row[0]},${row[1]},${row[2]},${row[3].toFixed(12)},${row[4].toFixed(
          5
        )},${row[5]},${row[6]}`
    );
    fs.writeFileSync(outPath, [header, ...csvLines].join("\n"), "utf8");
    console.log(`Wrote ${results.length} rows to ${outPath}`);
  });
  test("Apple Watch", () => {
    const dataPath = path.join(
      __dirname,
      "..",
      "fixtures",
      "data-apple-watch.csv"
    );
    const csvData = fs.readFileSync(dataPath, "utf8");
    const inputData = csvParse(csvData, {
      columns: true,
      skip_empty_lines: true,
      cast: true,
    });

    // Mirror monitor.js: create a slow and a fast filter and run the fixture through them
    const kfSlow = new KalmanFilter(0.77e-7, 0.15);
    const kfFast = new KalmanFilter(50 * 0.77e-7, 0.15);

    // Initialize with first sample timestamp as base time
    const firstTs = Math.floor(parseFloat(inputData[0].timestamp));
    kfSlow.init(parseFloat(inputData[0].value), firstTs);
    kfFast.init(parseFloat(inputData[0].value), firstTs);

    let lastTS = firstTs;
    const results = [];
    let mu = [0, 1]; // fast at first
    const p11 = 0.995;
    const p22 = 0.95;
    const Pi = [
      [p11, 1 - p11], // from slow -> [slow, fast]
      [1 - p22, p22], // from fast -> [slow, fast]
    ];

    inputData.forEach((row) => {
      const ts = Math.floor(parseFloat(row.timestamp));
      const val = parseFloat(row.value);

      const timestep = (ts - kfSlow.stateTS) / 1000;
      kfSlow.predict(ts);
      kfSlow.correct(val, ts);

      kfFast.predict(ts);
      kfFast.correct(val, ts);

      const logL = /** @type {number[]} */ ([
        kfSlow.logL ?? Number.NEGATIVE_INFINITY,
        kfFast.logL ?? Number.NEGATIVE_INFINITY,
      ]);

      const muPrior = matMul(transpose(Pi), [[mu[0]], [mu[1]]]).map(
        (v) => v[0]
      );
      // normalize and update mu (numerically stable, guard against under/overflow)
      const maxLogL = Math.max(...logL);
      let weights = muPrior.map((m, i) => m * Math.exp(logL[i] - maxLogL));
      let muSum = weights.reduce((a, b) => a + b, 0);
      if (!isFinite(muSum) || muSum === 0) {
        // fallback to uniform prior if weights collapsed
        weights = [0.5, 0.5];
        muSum = 1;
      }
      mu = weights.map((w) => w / muSum);

      const [meanSlow, kSlow] = /** @type {number[]} */ (
        kfSlow.mean() || [0, 0]
      );
      const [meanFast, kFast] = /** @type {number[]} */ (
        kfFast.mean() || [0, 0]
      );
      const fused = mu[0] * meanSlow + mu[1] * meanFast;
      const fusedK = mu[0] * kSlow + mu[1] * kFast;

      results.push([
        ts / 1000,
        val,
        fused,
        3600 * fusedK,
        mu[0],
        meanSlow,
        meanFast,
      ]);

      lastTS = ts;
    });

    // console.log(results);

    const outDir = path.join(__dirname, "..", "out");
    fs.mkdirSync(outDir, { recursive: true });
    const outPath = path.join(outDir, "applewatch.csv");
    const header = "timestamp,value,fused,k,mu,slow,fast";
    const csvLines = results.map(
      (row) =>
        `${row[0]},${row[1]},${row[2]},${row[3].toFixed(12)},${row[4].toFixed(
          5
        )},${row[5]},${row[6]}`
    );
    fs.writeFileSync(outPath, [header, ...csvLines].join("\n"), "utf8");
    console.log(`Wrote ${results.length} rows to ${outPath}`);
  });
  test("Ninebot", () => {
    const dataPath = path.join(__dirname, "..", "fixtures", "data-ninebot.csv");
    const csvData = fs.readFileSync(dataPath, "utf8");
    const inputData = csvParse(csvData, {
      columns: true,
      skip_empty_lines: true,
      cast: true,
    });

    // Mirror monitor.js: create a slow and a fast filter and run the fixture through them
    const kfSlow = new KalmanFilter(0.77e-7, 0.15);
    const kfFast = new KalmanFilter(50 * 0.77e-7, 0.15);

    // Initialize with first sample timestamp as base time
    const firstTs = Math.floor(parseFloat(inputData[0].timestamp));
    kfSlow.init(parseFloat(inputData[0].value), firstTs);
    kfFast.init(parseFloat(inputData[0].value), firstTs);

    let lastTS = firstTs;
    const results = [];
    let mu = [0, 1]; // fast at first
    const p11 = 0.995;
    const p22 = 0.95;
    const Pi = [
      [p11, 1 - p11], // from slow -> [slow, fast]
      [1 - p22, p22], // from fast -> [slow, fast]
    ];

    inputData.forEach((row) => {
      const ts = Math.floor(parseFloat(row.timestamp));
      const val = parseFloat(row.value);

      const timestep = (ts - kfSlow.stateTS) / 1000;
      kfSlow.predict(ts);
      kfSlow.correct(val, ts);

      kfFast.predict(ts);
      kfFast.correct(val, ts);

      const logL = /** @type {number[]} */ ([
        kfSlow.logL ?? Number.NEGATIVE_INFINITY,
        kfFast.logL ?? Number.NEGATIVE_INFINITY,
      ]);

      const muPrior = matMul(transpose(Pi), [[mu[0]], [mu[1]]]).map(
        (v) => v[0]
      );
      // normalize and update mu (numerically stable, guard against under/overflow)
      const maxLogL = Math.max(...logL);
      let weights = muPrior.map((m, i) => m * Math.exp(logL[i] - maxLogL));
      let muSum = weights.reduce((a, b) => a + b, 0);
      if (!isFinite(muSum) || muSum === 0) {
        // fallback to uniform prior if weights collapsed
        weights = [0.5, 0.5];
        muSum = 1;
      }
      mu = weights.map((w) => w / muSum);

      const [meanSlow, kSlow] = /** @type {number[]} */ (
        kfSlow.mean() || [0, 0]
      );
      const [meanFast, kFast] = /** @type {number[]} */ (
        kfFast.mean() || [0, 0]
      );
      const fused = mu[0] * meanSlow + mu[1] * meanFast;
      const fusedK = mu[0] * kSlow + mu[1] * kFast;

      results.push([
        ts / 1000,
        val,
        fused,
        3600 * fusedK,
        mu[0],
        meanSlow,
        meanFast,
      ]);

      lastTS = ts;
    });

    // console.log(results);

    const outDir = path.join(__dirname, "..", "out");
    fs.mkdirSync(outDir, { recursive: true });
    const outPath = path.join(outDir, "ninebot.csv");
    const header = "timestamp,value,fused,k,mu,slow,fast";
    const csvLines = results.map(
      (row) =>
        `${row[0]},${row[1]},${row[2]},${row[3].toFixed(12)},${row[4].toFixed(
          5
        )},${row[5]},${row[6]}`
    );
    fs.writeFileSync(outPath, [header, ...csvLines].join("\n"), "utf8");
    console.log(`Wrote ${results.length} rows to ${outPath}`);
  });
});
