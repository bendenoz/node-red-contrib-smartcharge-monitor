const { IMMFilter } = require("./imm-filter");

describe("IMMFilter", () => {
  it("initializes with fast mode active (mu = [0, 1])", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(10, 1000);

    expect(imm.mu).toEqual([0, 1]);
    expect(imm.mean()).toEqual([10, 0]);
  });

  it("predict-only ticks do not mutate previousCorrected or mu", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(50, 1000);
    imm.step(49.5, 21000);

    const prevSlowMean = imm.kfSlow.previousCorrected.mean.map((r) => [...r]);
    const prevFastMean = imm.kfFast.previousCorrected.mean.map((r) => [...r]);
    const prevMu = [...imm.mu];
    const prevCorrectTS = imm.correctTS;

    imm.predict(26500);
    imm.predict(32000);

    expect(imm.kfSlow.previousCorrected.mean).toEqual(prevSlowMean);
    expect(imm.kfFast.previousCorrected.mean).toEqual(prevFastMean);
    expect(imm.mu).toEqual(prevMu);
    expect(imm.correctTS).toBe(prevCorrectTS);
    expect(imm.stateTS).toBe(32000);
  });

  it("handles dt <= 0 without NaN or singular covariance", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(20, 10000);

    // dt == 0
    imm.step(20.1, 10000);
    // dt < 0
    imm.step(19.9, 5000);

    const [val, k] = imm.mean();
    expect(Number.isFinite(val)).toBe(true);
    expect(Number.isFinite(k)).toBe(true);
    expect(Number.isFinite(imm.kfSlow.logL)).toBe(true);
    expect(Number.isFinite(imm.kfFast.logL)).toBe(true);
    expect(imm.mu[0] + imm.mu[1]).toBeCloseTo(1, 10);
  });

  it("maintains symmetric positive-definite covariances and normalized mu across steps", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(50, 0);

    for (let i = 1; i <= 30; i++) {
      const ts = i * 20000;
      // flat then decaying
      const val = i < 15 ? 50 : 50 * Math.exp(-0.0005 * (i - 15) * 20);
      imm.step(val, ts);

      expect(imm.mu[0] + imm.mu[1]).toBeCloseTo(1, 10);
      for (const kf of [imm.kfSlow, imm.kfFast]) {
        const P = kf.previousCorrected.covariance;
        expect(P[0][0]).toBeGreaterThan(0);
        expect(P[0][0] * P[1][1] - P[0][1] * P[1][0]).toBeGreaterThan(0);
        expect(Math.abs(P[0][1] - P[1][0])).toBeLessThan(1e-12);
      }
    }
  });
});
