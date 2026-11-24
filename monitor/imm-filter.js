"use strict";

const { KalmanFilter } = require("./kalman-filter");

const DEFAULT_P11 = 0.995; // ~ 1 - deltaT / tauSlow
const DEFAULT_P22 = 0.925; // ~ 1 - deltaT / tauFast
const DEFAULT_PI = [
  [DEFAULT_P11, 1 - DEFAULT_P11], // from slow -> [slow, fast]
  [1 - DEFAULT_P22, DEFAULT_P22], // from fast -> [slow, fast]
];

class IMMFilter {
  /** @type {KalmanFilter} */
  kfSlow;

  /** @type {KalmanFilter} */
  kfFast;

  /** @type {[[number, number], [number, number]]} */
  Pi;

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
   * @param {[[number, number], [number, number]]} [opts.Pi] - Markov transition matrix
   */
  constructor({ stdK, pwrStdev, Pi = /** @type {[[number, number], [number, number]]} */ (DEFAULT_PI) }) {
    this.kfSlow = new KalmanFilter(stdK, pwrStdev);
    this.kfFast = new KalmanFilter(50 * stdK, pwrStdev);
    this.Pi = Pi;
  }

  /**
   * Initialize both mode filters
   * @param {number} initValue
   * @param {number} initTs
   */
  init(initValue, initTs) {
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
    const filters = [this.kfSlow, this.kfFast];

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
    const prevStates = filters.map((f) => f.previousCorrected);
    if (prevStates[0] && prevStates[1]) {
      /** @type {number[][][]} */
      const mixedMeans = [];
      /** @type {number[][][]} */
      const mixedCovs = [];

      for (let j = 0; j < 2; j++) {
        const w = mixingWeights[j];
        const m0 = prevStates[0].mean;
        const m1 = prevStates[1].mean;
        const mixedMean = [
          [w[0] * m0[0][0] + w[1] * m1[0][0]],
          [w[0] * m0[1][0] + w[1] * m1[1][0]],
        ];

        const mixedCov = [
          [0, 0],
          [0, 0],
        ];
        for (let i = 0; i < 2; i++) {
          const mi = prevStates[i].mean;
          const Pi_cov = prevStates[i].covariance;
          const d0 = mi[0][0] - mixedMean[0][0];
          const d1 = mi[1][0] - mixedMean[1][0];

          mixedCov[0][0] += w[i] * (Pi_cov[0][0] + d0 * d0);
          mixedCov[0][1] += w[i] * (Pi_cov[0][1] + d0 * d1);
          mixedCov[1][0] += w[i] * (Pi_cov[1][0] + d1 * d0);
          mixedCov[1][1] += w[i] * (Pi_cov[1][1] + d1 * d1);
        }

        // Enforce exact symmetry against floating-point drift
        const symOff = 0.5 * (mixedCov[0][1] + mixedCov[1][0]);
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
};
