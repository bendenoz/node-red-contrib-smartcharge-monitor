const { IMMFilter } = require("./imm-filter");
const { KalmanFilter } = require("./kalman-filter");

describe("IMMFilter", () => {
  it("returns [null, null] before init and throws on predict/step without mutating Pi", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    const initialPi = imm.Pi.map((r) => [...r]);
    expect(imm.mean()).toEqual([null, null]);
    expect(() => imm.predict(1000)).toThrow("No KF instance");
    expect(() => imm.step(10, 1000)).toThrow("No KF instance");
    expect(imm.Pi).toEqual(initialPi);
  });

  it("initializes with fast mode active (mu = [0, 1])", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(10, 1000);

    expect(imm.mu).toEqual([0, 1]);
    expect(imm.mean()).toEqual([10, 0]);
  });

  it("performs exact 2-mode IMM mixing including spread-of-the-means covariance", () => {
    const Pi = [
      [0.9, 0.1],
      [0.2, 0.8],
    ];
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15, Pi });
    imm.init(50, 0);

    // Give slow and fast distinct corrected means and covariances
    imm.mu = [0.6, 0.4];
    const m0 = [[50], [1e-4]];
    const m1 = [[48], [5e-4]];
    const P0 = [
      [1.0, 0.0005],
      [0.0005, 1e-6],
    ];
    const P1 = [
      [2.0, -0.001],
      [-0.001, 4e-6],
    ];
    imm.kfSlow.setPreviousCorrected(m0, P0);
    imm.kfFast.setPreviousCorrected(m1, P1);

    // Analytical expected mixing inputs
    const muPrior = [0.9 * 0.6 + 0.2 * 0.4, 0.1 * 0.6 + 0.8 * 0.4]; // [0.62, 0.38]
    const wSlow = [(0.9 * 0.6) / muPrior[0], (0.2 * 0.4) / muPrior[0]];
    const expectedMixedMean0 = [
      wSlow[0] * 50 + wSlow[1] * 48,
      wSlow[0] * 1e-4 + wSlow[1] * 5e-4,
    ];
    const d0 = [50 - expectedMixedMean0[0], 1e-4 - expectedMixedMean0[1]];
    const d1 = [48 - expectedMixedMean0[0], 5e-4 - expectedMixedMean0[1]];
    const expectedMixedP00 =
      wSlow[0] * (P0[0][0] + d0[0] * d0[0]) +
      wSlow[1] * (P1[0][0] + d1[0] * d1[0]);

    // Spy on kfSlow.predict to capture the mixed previousCorrected passed into it
    const origPredict = imm.kfSlow.predict.bind(imm.kfSlow);
    let capturedMixedMean = /** @type {number[][]} */ ([]);
    let capturedMixedCov = /** @type {number[][]} */ ([]);
    jest.spyOn(imm.kfSlow, "predict").mockImplementation((ts, nf) => {
      if (!imm.kfSlow.previousCorrected) throw new Error("Expected previousCorrected");
      capturedMixedMean = imm.kfSlow.previousCorrected.mean.map((r) => [...r]);
      capturedMixedCov = imm.kfSlow.previousCorrected.covariance.map((r) => [
        ...r,
      ]);
      return origPredict(ts, nf);
    });

    imm.step(47.5, 20000);

    expect(capturedMixedMean[0][0]).toBeCloseTo(expectedMixedMean0[0], 10);
    expect(capturedMixedMean[1][0]).toBeCloseTo(expectedMixedMean0[1], 12);
    expect(capturedMixedCov[0][0]).toBeCloseTo(expectedMixedP00, 10);
    // Spread-of-the-means strictly increases variance beyond linear combination
    expect(capturedMixedCov[0][0]).toBeGreaterThan(
      wSlow[0] * P0[0][0] + wSlow[1] * P1[0][0]
    );
  });

  it("predict-only ticks extrapolate mean without mutating previousCorrected, mu, or future step()", () => {
    const immA = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    const immB = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    immA.init(50, 0);
    immB.init(50, 0);

    // Feed decaying samples so k > 0
    for (let i = 1; i <= 5; i++) {
      const ts = i * 20000;
      const val = 50 * Math.exp(-0.0005 * (ts / 1000));
      immA.step(val, ts);
      immB.step(val, ts);
    }

    if (!immA.kfSlow.previousCorrected || !immA.kfFast.previousCorrected) {
      throw new Error("Expected previousCorrected");
    }
    const prevSlowMean = immA.kfSlow.previousCorrected.mean.map((r) => [...r]);
    const prevSlowCov = immA.kfSlow.previousCorrected.covariance.map((r) => [
      ...r,
    ]);
    const prevFastMean = immA.kfFast.previousCorrected.mean.map((r) => [...r]);
    const prevFastCov = immA.kfFast.previousCorrected.covariance.map((r) => [
      ...r,
    ]);
    const prevMu = [...immA.mu];
    const prevCorrectTS = immA.correctTS;
    const [valBefore] = immA.mean();

    immA.predict(105500);
    immA.predict(111000);

    const [valExtrapolated, kExtrapolated] = immA.mean();
    expect(valBefore).not.toBeNull();
    expect(valExtrapolated).not.toBeNull();
    expect(kExtrapolated).toBeGreaterThan(0);
    expect(/** @type {number} */ (valExtrapolated)).toBeLessThan(
      /** @type {number} */ (valBefore)
    );

    expect(immA.kfSlow.previousCorrected.mean).toEqual(prevSlowMean);
    expect(immA.kfSlow.previousCorrected.covariance).toEqual(prevSlowCov);
    expect(immA.kfFast.previousCorrected.mean).toEqual(prevFastMean);
    expect(immA.kfFast.previousCorrected.covariance).toEqual(prevFastCov);
    expect(immA.mu).toEqual(prevMu);
    expect(immA.correctTS).toBe(prevCorrectTS);
    expect(immA.stateTS).toBe(111000);

    // Next step() matches an identical filter that had no intermediate predict() ticks
    immA.step(47.0, 120000);
    immB.step(47.0, 120000);
    expect(immA.mean()[0]).toBeCloseTo(/** @type {number} */ (immB.mean()[0]), 12);
    expect(immA.mean()[1]).toBeCloseTo(/** @type {number} */ (immB.mean()[1]), 12);
    expect(immA.mu[0]).toBeCloseTo(immB.mu[0], 12);
  });

  it("handles dt <= 0 and initTs = 0 with monotonic timestamps and valid covariances", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(20, 0);

    imm.predict(0);
    imm.predict(-500);
    expect(imm.stateTS).toBe(0);

    // dt == 0
    imm.step(20.1, 0);
    expect(imm.correctTS).toBe(0);
    expect(imm.stateTS).toBe(0);

    // dt > 0 then dt < 0
    imm.step(20.0, 10000);
    imm.step(19.9, 5000);
    expect(imm.correctTS).toBe(10000);
    expect(imm.stateTS).toBe(10000);

    const [val, k] = imm.mean();
    expect(Number.isFinite(val)).toBe(true);
    expect(Number.isFinite(k)).toBe(true);
    expect(Number.isFinite(imm.kfSlow.logL)).toBe(true);
    expect(Number.isFinite(imm.kfFast.logL)).toBe(true);
    expect(imm.mu[0] + imm.mu[1]).toBeCloseTo(1, 10);

    for (const kf of [imm.kfSlow, imm.kfFast]) {
      if (!kf.previousCorrected) throw new Error("Expected previousCorrected");
      const P = kf.previousCorrected.covariance;
      expect(P[0][0]).toBeGreaterThan(0);
      expect(P[0][0] * P[1][1] - P[0][1] * P[1][0]).toBeGreaterThan(0);
      expect(Math.abs(P[0][1] - P[1][0])).toBeLessThan(1e-12);
    }
  });

  it("handles pwrStdev === 0 across dt = 0 and noisy decaying dt > 0 trajectories without throwing", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0 });
    imm.init(10, 1000);
    imm.step(10, 1000);
    imm.step(10, 1000); // S = [[0]]

    // Decaying noisy curve that previously triggered subnormal covariance Cauchy-Schwarz errors
    imm.init(50, 0);
    for (let i = 1; i <= 30; i++) {
      const val = 50 * Math.exp(-0.0005 * i * 20) + (i % 2 ? 0.05 : -0.05);
      imm.step(val, i * 20000);
    }

    const [val, k] = imm.mean();
    expect(Number.isFinite(val)).toBe(true);
    expect(Number.isFinite(k)).toBe(true);
    expect(imm.mu[0] + imm.mu[1]).toBeCloseTo(1, 10);
  });

  it("rejects non-finite inputs without mutating filter state", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(25, 1000);
    imm.step(25, 21000);

    const snapshotMean = imm.mean();
    const snapshotMu = [...imm.mu];
    const snapshotTS = imm.correctTS;

    expect(() => imm.predict(NaN)).toThrow(TypeError);
    expect(() => imm.step(NaN, 41000)).toThrow(TypeError);
    expect(() => imm.step(Infinity, 41000)).toThrow(TypeError);
    expect(() => imm.step(25, Infinity)).toThrow(TypeError);

    expect(imm.mean()).toEqual(snapshotMean);
    expect(imm.mu).toEqual(snapshotMu);
    expect(imm.correctTS).toBe(snapshotTS);

    const kf = new KalmanFilter(0.77e-7, 0.15);
    expect(() => kf.init(NaN, 0)).toThrow(TypeError);
    kf.init(10, 0);
    expect(() => kf.predict(Infinity)).toThrow(TypeError);
    expect(() => kf.correct(NaN, 1000)).toThrow(TypeError);
  });

  it("shifts mode probability from slow steady state to fast during decay onset", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(50, 0);

    // Steady constant-current phase: slow mode should dominate
    for (let i = 1; i <= 40; i++) {
      imm.step(50, i * 20000);
    }
    const muSlowSteady = imm.mu[0];
    expect(muSlowSteady).toBeGreaterThan(0.8);

    // Abrupt onset of exponential decay: fast mode probability rises at the knee
    const kTrue = 2 / 3600; // 2 h^-1
    let maxMuFast = 0;
    for (let i = 41; i <= 60; i++) {
      const dtSec = (i - 40) * 20;
      imm.step(50 * Math.exp(-kTrue * dtSec), i * 20000);
      if (imm.mu[1] > maxMuFast) maxMuFast = imm.mu[1];
    }
    expect(maxMuFast).toBeGreaterThan(0.5);
    const [, kAfter] = imm.mean();
    expect(/** @type {number} */ (kAfter) * 3600).toBeGreaterThan(0.5);
  });

  it("falls back to uniform weights if muPrior collapses to zero", () => {
    const zeroPi = [
      [0, 0],
      [0, 0],
    ];
    const imm = new IMMFilter({
      stdK: 0.77e-7,
      pwrStdev: 0.15,
      Pi: /** @type {[[number, number], [number, number]]} */ (zeroPi),
    });
    imm.init(10, 0);
    imm.step(10, 20000);

    expect(imm.mu).toEqual([0.5, 0.5]);
  });

  it("maintains symmetric positive-definite covariances and normalized mu across steps", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(50, 0);

    for (let i = 1; i <= 30; i++) {
      const ts = i * 20000;
      const val = i < 15 ? 50 : 50 * Math.exp(-0.0005 * (i - 15) * 20);
      imm.step(val, ts);

      expect(imm.mu[0] + imm.mu[1]).toBeCloseTo(1, 10);
      for (const kf of [imm.kfSlow, imm.kfFast]) {
        if (!kf.previousCorrected) throw new Error("Expected previousCorrected");
        const P = kf.previousCorrected.covariance;
        expect(P[0][0]).toBeGreaterThan(0);
        expect(P[0][0] * P[1][1] - P[0][1] * P[1][0]).toBeGreaterThan(0);
        expect(Math.abs(P[0][1] - P[1][0])).toBeLessThan(1e-12);
      }
    }
  });

  it("scales Pi via exact 2-state CTMC exp(Q*dt) without mode inversion on large gaps", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    imm.init(50, 0);

    // dt == 0 -> identity transition
    imm.step(50, 0);
    expect(imm.Pi).toEqual([
      [1, 0],
      [0, 1],
    ]);

    // dt == 20 s -> matches ~0.995 and ~0.926
    imm.step(50, 20000);
    expect(imm.Pi[0][0]).toBeCloseTo(0.995, 3);
    expect(imm.Pi[1][1]).toBeCloseTo(0.926, 3);

    // dt == 10000 s -> both rows approach stationary distribution [4000/4260, 260/4260]
    imm.step(50, 10020000);
    const statSlow = 4000 / 4260;
    const statFast = 260 / 4260;
    expect(imm.Pi[0][0]).toBeCloseTo(statSlow, 4);
    expect(imm.Pi[1][0]).toBeCloseTo(statSlow, 4);
    expect(imm.Pi[1][1]).toBeCloseTo(statFast, 4);

    // Invalid tau throws
    expect(
      () => new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15, tauSlow: 0 })
    ).toThrow(TypeError);
  });

  it("computes mixture posterior kStd() (including spread-of-means) and kZScore()", () => {
    const imm = new IMMFilter({ stdK: 0.77e-7, pwrStdev: 0.15 });
    expect(imm.kStd()).toBeNull();
    expect(imm.kZScore(1 / 6.66)).toBe(0);

    imm.init(50, 0);
    imm.mu = [0.7, 0.3];
    if (!imm.kfSlow.state || !imm.kfFast.state) {
      throw new Error("Expected states after init");
    }
    imm.kfSlow.state.mean = [[50], [2e-4]];
    imm.kfFast.state.mean = [[50], [6e-4]];
    imm.kfSlow.state.covariance = [
      [0.1, 0],
      [0, 1e-10],
    ];
    imm.kfFast.state.covariance = [
      [0.1, 0],
      [0, 4e-10],
    ];

    const kMean = 0.7 * 2e-4 + 0.3 * 6e-4;
    const expectedVar =
      0.7 * (1e-10 + (2e-4 - kMean) ** 2) + 0.3 * (4e-10 + (6e-4 - kMean) ** 2);
    const expectedStd = Math.sqrt(expectedVar);

    expect(imm.kStd()).toBeCloseTo(expectedStd, 12);
    expect(imm.kZScore(1 / 6.66)).toBeCloseTo(
      (kMean - 1 / 6.66 / 3600) / expectedStd,
      10
    );

    // Zero variance boundary handling
    imm.mu = [1, 0];
    imm.kfSlow.state.covariance[1][1] = 0;
    imm.kfSlow.state.mean[1][0] = 2e-4;
    expect(imm.kZScore(0)).toBe(Number.POSITIVE_INFINITY);
    expect(imm.kZScore(10)).toBe(Number.NEGATIVE_INFINITY);
  });
});
