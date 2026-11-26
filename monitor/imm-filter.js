"use strict";

const { KalmanFilter } = require("./kalman-filter");

const DEFAULT_TAU_SLOW = 4000; // s (~66.7 min, p11 ~ 0.995 at dt = 20 s)
const DEFAULT_TAU_FAST = 260; // s (~4.3 min, p22 ~ 0.926 at dt = 20 s)

/**
 * Compute exact 2-state continuous-time Markov chain (CTMC) transition matrix exp(Q * dt)
 * with transition rates lambda_12 = 1 / tauSlow and lambda_21 = 1 / tauFast.
 * @param {number} dt
 * @param {number} [tauSlow=DEFAULT_TAU_SLOW]
 * @param {number} [tauFast=DEFAULT_TAU_FAST]
 * @return {[[number, number], [number, number]]}
 */
function transitionMatrix(
  dt,
  tauSlow = DEFAULT_TAU_SLOW,
  tauFast = DEFAULT_TAU_FAST
) {
  const safeDt = dt > 0 ? dt : 0;
  const lambda12 = 1 / tauSlow;
  const lambda21 = 1 / tauFast;
  const totalRate = lambda12 + lambda21;
  const decay = Math.exp(-totalRate * safeDt);
  const p11 = (lambda21 + lambda12 * decay) / totalRate;
  const p22 = (lambda12 + lambda21 * decay) / totalRate;
  return [
    [p11, 1 - p11], // from slow -> [slow, fast]
    [1 - p22, p22], // from fast -> [slow, fast]
  ];
}

const DEFAULT_PI = transitionMatrix(20, DEFAULT_TAU_SLOW, DEFAULT_TAU_FAST);

class IMMFilter {
  /** @type {KalmanFilter} */
  kfSlow;

  /** @type {KalmanFilter} */
  kfFast;

  /** @type {[[number, number], [number, number]]} */
  Pi;

  /** @type {boolean} */
  fixedPi;

  /** @type {number} */
  tauSlow;

  /** @type {number} */
  tauFast;

  /** @type {[number, number]} */
  mu = [0, 1];

  /** @type {number} */
  stateTS = 0;

  /** @type {number} */
  correctTS = 0;

  /**
   * @param {object} opts
   * @param {number} opts.stdK - Base process noise std dev for slow mode
   * @param {number} opts.pwrStdev - Measurement noise std dev
   * @param {[[number, number], [number, number]]} [opts.Pi] - Optional fixed Markov transition matrix
   * @param {number} [opts.tauSlow=DEFAULT_TAU_SLOW] - Slow mode dwell time constant (seconds)
   * @param {number} [opts.tauFast=DEFAULT_TAU_FAST] - Fast mode dwell time constant (seconds)
   */
  constructor({
    stdK,
    pwrStdev,
    Pi,
    tauSlow = DEFAULT_TAU_SLOW,
    tauFast = DEFAULT_TAU_FAST,
  }) {
    if (!Number.isFinite(tauSlow) || tauSlow <= 0 || !Number.isFinite(tauFast) || tauFast <= 0) {
      throw new TypeError("tauSlow and tauFast must be finite positive numbers");
    }
    this.kfSlow = new KalmanFilter(stdK, pwrStdev);
    this.kfFast = new KalmanFilter(50 * stdK, pwrStdev);
    this.tauSlow = tauSlow;
    this.tauFast = tauFast;
    this.fixedPi = Boolean(Pi);
    const basePi = Pi || DEFAULT_PI;
    this.Pi = [
      [basePi[0][0], basePi[0][1]],
      [basePi[1][0], basePi[1][1]],
    ];
  }

  /**
   * Initialize both mode filters
   * @param {number} initValue
   * @param {number} initTs
   */
  init(initValue, initTs) {
    if (!Number.isFinite(initValue) || !Number.isFinite(initTs)) {
      throw new TypeError("initValue and initTs must be finite numbers");
    }
    this.kfSlow.init(initValue, initTs);
    this.kfFast.init(initValue, initTs);
    this.mu = [0, 1]; // fast at first
    this.stateTS = initTs;
    this.correctTS = initTs;
  }

  /**
   * Predict-only tick: extrapolates from the last corrected mode states
   * without mutating previousCorrected, mu, or performing mixing.
   * @param {number} ts - Timestamp in milliseconds
   */
  predict(ts) {
    if (!Number.isFinite(ts)) {
      throw new TypeError("ts must be a finite number");
    }
    this.kfSlow.predict(ts);
    this.kfFast.predict(ts);
    this.stateTS = Math.max(ts, this.correctTS);
  }

  /**
   * Full IMM measurement update: mixing -> predict -> correct -> mu update
   * @param {number} value
   * @param {number} ts - Timestamp in milliseconds
   */
  step(value, ts) {
    if (!Number.isFinite(value) || !Number.isFinite(ts)) {
      throw new TypeError("value and ts must be finite numbers");
    }
    if (!this.kfSlow.kf || !this.kfFast.kf) {
      throw new Error("No KF instance");
    }
    const filters = [this.kfSlow, this.kfFast];
    if (!this.fixedPi) {
      const dt = Math.max(0, (ts - this.correctTS) / 1000);
      this.Pi = transitionMatrix(dt, this.tauSlow, this.tauFast);
    }

    // 1. Predicted mode probabilities c_j = sum_i Pi[i][j] * mu[i]
    const muPrior = [
      this.Pi[0][0] * this.mu[0] + this.Pi[1][0] * this.mu[1],
      this.Pi[0][1] * this.mu[0] + this.Pi[1][1] * this.mu[1],
    ];

    // Mixing weights mu_{i|j} = Pi[i][j] * mu[i] / c_j
    const mixingWeights = [0, 1].map((j) => {
      const cj = muPrior[j];
      if (!Number.isFinite(cj) || cj <= 0) return [0.5, 0.5];
      return [
        (this.Pi[0][j] * this.mu[0]) / cj,
        (this.Pi[1][j] * this.mu[1]) / cj,
      ];
    });

    // 2. State & covariance mixing from previousCorrected
    const prev0 = this.kfSlow.previousCorrected;
    const prev1 = this.kfFast.previousCorrected;
    if (prev0 && prev1) {
      const prevPair = [prev0, prev1];
      /** @type {number[][][]} */
      const mixedMeans = [];
      /** @type {number[][][]} */
      const mixedCovs = [];

      for (let j = 0; j < 2; j++) {
        const w = mixingWeights[j];
        const m0 = prev0.mean;
        const m1 = prev1.mean;
        const mixedMean = [
          [w[0] * m0[0][0] + w[1] * m1[0][0]],
          [w[0] * m0[1][0] + w[1] * m1[1][0]],
        ];

        const mixedCov = [
          [0, 0],
          [0, 0],
        ];
        for (let i = 0; i < 2; i++) {
          const mi = prevPair[i].mean;
          const Pi_cov = prevPair[i].covariance;
          const d0 = mi[0][0] - mixedMean[0][0];
          const d1 = mi[1][0] - mixedMean[1][0];

          mixedCov[0][0] += w[i] * (Pi_cov[0][0] + d0 * d0);
          mixedCov[0][1] += w[i] * (Pi_cov[0][1] + d0 * d1);
          mixedCov[1][0] += w[i] * (Pi_cov[1][0] + d1 * d0);
          mixedCov[1][1] += w[i] * (Pi_cov[1][1] + d1 * d1);
        }

        // Enforce symmetry, Cauchy-Schwarz, and avoid subnormal (< 1e-150) underflow in p * p
        mixedCov[0][0] = mixedCov[0][0] >= 1e-150 ? mixedCov[0][0] : 0;
        mixedCov[1][1] = mixedCov[1][1] >= 1e-150 ? mixedCov[1][1] : 0;
        const maxOff = Math.sqrt(mixedCov[0][0] * mixedCov[1][1]);
        let symOff = 0.5 * (mixedCov[0][1] + mixedCov[1][0]);
        if (Math.abs(symOff) > maxOff) {
          symOff = symOff < 0 ? -maxOff : maxOff;
        }
        mixedCov[0][1] = symOff;
        mixedCov[1][0] = symOff;

        mixedMeans.push(mixedMean);
        mixedCovs.push(mixedCov);
      }

      filters[0].setPreviousCorrected(mixedMeans[0], mixedCovs[0]);
      filters[1].setPreviousCorrected(mixedMeans[1], mixedCovs[1]);
    }

    // 3. Mode-matched predict & correct
    for (let j = 0; j < 2; j++) {
      filters[j].predict(ts);
      filters[j].correct(value, ts);
    }

    // 4. Mode probability update
    const logL = [
      filters[0].logL ?? Number.NEGATIVE_INFINITY,
      filters[1].logL ?? Number.NEGATIVE_INFINITY,
    ];
    const maxLogL = Math.max(logL[0], logL[1]);
    let weights = [
      muPrior[0] * Math.exp(logL[0] - maxLogL),
      muPrior[1] * Math.exp(logL[1] - maxLogL),
    ];
    let muSum = weights[0] + weights[1];
    if (!Number.isFinite(muSum) || muSum <= 0) {
      weights = [0.5, 0.5];
      muSum = 1;
    }
    this.mu = [weights[0] / muSum, weights[1] / muSum];

    const effectiveTs = Math.max(ts, this.correctTS);
    this.correctTS = effectiveTs;
    this.stateTS = effectiveTs;
  }

  /**
   * Innovation diagnostics from the fast mode filter
   * @return {{ nis: number, innovation: number }}
   */
  fastInnovation() {
    return {
      nis: this.kfFast.nis ?? 0,
      innovation: this.kfFast.innovation ?? 0,
    };
  }

  /**
   * Fused [power, k] estimate weighted by mode probabilities mu
   * @return {[number | null, number | null]}
   */
  mean() {
    const [valSlow, kSlow] = this.kfSlow.mean();
    const [valFast, kFast] = this.kfFast.mean();
    if (
      valSlow === null ||
      kSlow === null ||
      valFast === null ||
      kFast === null
    ) {
      return [null, null];
    }
    return [
      this.mu[0] * valSlow + this.mu[1] * valFast,
      this.mu[0] * kSlow + this.mu[1] * kFast,
    ];
  }
}

module.exports = {
  IMMFilter,
  DEFAULT_PI,
  DEFAULT_TAU_SLOW,
  DEFAULT_TAU_FAST,
  transitionMatrix,
};
