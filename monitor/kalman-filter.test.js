const fs = require("fs");
const path = require("path");
const { parse: csvParse } = require("csv-parse/sync");

const { IMMFilter } = require("./imm-filter");

const STD_K = 0.77e-7;
const PWR_STDEV = 0.15;

/**
 * Run a CSV charging fixture through IMMFilter and assert filter invariants
 * on every step. Optionally writes debug CSVs when WRITE_DEBUG_CSV=1.
 * @param {string} fixtureFile
 * @param {string} outFileName
 */
function runFixtureWithIMM(fixtureFile, outFileName) {
  const dataPath = path.join(__dirname, "..", "fixtures", fixtureFile);
  const csvData = fs.readFileSync(dataPath, "utf8");
  const inputData = csvParse(csvData, {
    columns: true,
    skip_empty_lines: true,
    cast: true,
  });

  const imm = new IMMFilter({ stdK: STD_K, pwrStdev: PWR_STDEV });
  const firstTs = Math.floor(parseFloat(inputData[0].timestamp));
  imm.init(parseFloat(inputData[0].value), firstTs);

  const results = [];
  let resets = 0;

  inputData.forEach((row) => {
    const ts = Math.floor(parseFloat(row.timestamp));
    const val = parseFloat(row.value);

    const [prevVal] = imm.mean();
    if (prevVal === null || (prevVal + val && !(prevVal * val))) {
      imm.init(val, ts);
      resets++;
    } else {
      imm.step(val, ts);
      const { nis, innovation } = imm.fastInnovation();
      const mahalanobis = Math.max(0, nis) ** 0.5;
      if (mahalanobis > 3 && Math.abs(innovation) >= 0.2 * Math.abs(val)) {
        imm.init(val, ts);
        resets++;
      }
    }

    // Check IMM invariants on every sample
    expect(imm.mu[0] + imm.mu[1]).toBeCloseTo(1, 10);
    expect(imm.mu[0]).toBeGreaterThanOrEqual(0);
    expect(imm.mu[0]).toBeLessThanOrEqual(1);
    expect(imm.mu[1]).toBeGreaterThanOrEqual(0);
    expect(imm.mu[1]).toBeLessThanOrEqual(1);

    const [fused, fusedK] = imm.mean();
    const [meanSlow] = imm.kfSlow.mean();
    const [meanFast] = imm.kfFast.mean();
    expect(Number.isFinite(fused)).toBe(true);
    expect(Number.isFinite(fusedK)).toBe(true);
    expect(Number.isFinite(meanSlow)).toBe(true);
    expect(Number.isFinite(meanFast)).toBe(true);

    for (const kf of [imm.kfSlow, imm.kfFast]) {
      const P = kf.previousCorrected.covariance;
      expect(P[0][0]).toBeGreaterThan(0);
      expect(P[0][0] * P[1][1] - P[0][1] * P[1][0]).toBeGreaterThan(0);
      expect(Math.abs(P[0][1] - P[1][0])).toBeLessThan(1e-12);
    }

    results.push([
      ts / 1000,
      val,
      /** @type {number} */ (fused),
      3600 * /** @type {number} */ (fusedK),
      imm.mu[0],
      /** @type {number} */ (meanSlow),
      /** @type {number} */ (meanFast),
    ]);
  });

  if (process.env.WRITE_DEBUG_CSV === "1") {
    const outDir = path.join(__dirname, "..", "out");
    fs.mkdirSync(outDir, { recursive: true });
    const outPath = path.join(outDir, outFileName);
    const header = "timestamp,value,fused,k,mu,slow,fast";
    const csvLines = results.map(
      (row) =>
        `${row[0]},${row[1]},${row[2]},${row[3].toFixed(12)},${row[4].toFixed(
          5
        )},${row[5]},${row[6]}`
    );
    fs.writeFileSync(outPath, [header, ...csvLines].join("\n"), "utf8");
  }

  return { results, resets };
}

describe("IMM fixture invariant tests", () => {
  test("Power Bank", () => {
    const { results, resets } = runFixtureWithIMM(
      "data-noisy-powerbank.csv",
      "powerbank.csv"
    );
    expect(results.length).toBe(1085);
    expect(resets).toBeGreaterThanOrEqual(2);
  });

  test("Apple Watch", () => {
    const { results, resets } = runFixtureWithIMM(
      "data-apple-watch.csv",
      "applewatch.csv"
    );
    expect(results.length).toBe(336);
    expect(resets).toBeGreaterThanOrEqual(1);
  });

  test("Ninebot", () => {
    const { results, resets } = runFixtureWithIMM(
      "data-ninebot.csv",
      "ninebot.csv"
    );
    expect(results.length).toBe(308);
    expect(resets).toBeGreaterThanOrEqual(2);
  });

  test("Samsung Watch", () => {
    const { results, resets } = runFixtureWithIMM(
      "data-samsung-watch.csv",
      "samsungwatch.csv"
    );
    expect(results.length).toBe(288);
    expect(resets).toBeGreaterThanOrEqual(1);
  });
});
