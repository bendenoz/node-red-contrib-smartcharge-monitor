"use strict";

const KalmanClass = require("kalman-filter/lib/kalman-filter");
const StateType = require("kalman-filter/lib/state");
const {
  subtract: sub,
  add,
  matMul,
  transpose,
  invert,
} = require("simple-linalg");

require("kalman-filter"); // must be required to init default models

class KalmanFilter {
  /** @type {KalmanClass | undefined} */
  kf;

  /** @type {StateType | null} */
  state = null;

  /** @type {StateType | null} */
  previousCorrected = null;

  /**
   * innovation from last correct() call
   * @type {number | null}
   */
  innovation = null;

  /**
   * log-likelihood from last correct() call
   * @type {number | null}
   */
  logL = null;

  /**
   * Normalized Innovation Squared from last correct() call
   * @type {number | null}
   */
  nis = null;

  /**
   * previous timestamp, in milliseconds
   * @type {number}
   */
  stateTS = 0;

  /**
   * previous timestamp, in milliseconds
   * @type {number}
   */
  correctTS = 0;

  /**
   * k noise - in units per second per square-rooted second (not units^2)
   * @type {number}
   */
  kStdev;

  /**
   * pwrStdev - power standard deviation
   * @type {number}
   */
  pwrStdev;

  /**
   * @param {number} kStdev process std dev
   * @param {number} pwrStdev power std dev
   */
  constructor(kStdev, pwrStdev) {
    this.kStdev = kStdev;
    this.pwrStdev = pwrStdev;
  }

  /**
   * Instanciate kalman filter with value
   * @param {number} initValue
   * @param {number} initTs
   */
  init(initValue, initTs) {
    if (!Number.isFinite(initValue) || !Number.isFinite(initTs)) {
      throw new TypeError("initValue and initTs must be finite numbers");
    }
    try {
      this.kf = new KalmanClass({
        observation: {
          dimension: 1,
          // R
          covariance: [[this.pwrStdev ** 2]],
        },
        dynamic: {
          // name: "exponential decay",
          dimension: 2,
          init: {
            mean: [[initValue], [0]],
            // Initial P
            covariance: [100 ** 2, /* (this.kStdev / 10) *  */ 1e-3 ** 2],
            index: -1,
          },
          transition: ({ previousCorrected, timestep }) => {
            const [[pwr]] = previousCorrected.mean;
            const target = 0;
            return [
              [1, (target - pwr) * timestep],
              [0, 1],
            ];
          },
          // Q (noise)
          covariance: ({
            previousCorrected: {
              mean: [[pwr]],
            },
            timestep,
            noiseFactor,
          }) => {
            const target = 0;

            // continuous white acceleration noise (CWNA)
            const qNoise = (noiseFactor * this.kStdev) ** 2;
            const C = target - pwr;
            const pwrVar = (qNoise * C ** 2 * timestep ** 3) / 3;
            const correlVal = (qNoise * C * timestep ** 2) / 2;
            const kVar = qNoise * timestep;
            return [
              [pwrVar, correlVal],
              [correlVal, kVar],
            ];
          },
        },
      });
    } catch (e) {
      console.error(e);
      throw e;
    }
    this.state = this.kf.getInitState();
    this.stateTS = initTs;
    this.previousCorrected = this.state;
    this.correctTS = initTs;
    this.innovation = null;
    this.logL = null;
    this.nis = null;
  }

  /**
   * Replace previousCorrected state (used by IMM mixing)
   * @param {number[][]} mean
   * @param {number[][]} covariance
   */
  setPreviousCorrected(mean, covariance) {
    this.previousCorrected = new StateType({
      mean,
      covariance,
      index: this.previousCorrected ? this.previousCorrected.index : -1,
    });
  }

  /**
   * Predict the next state
   * @param {number} ts - Timestamp in milliseconds
   * @param {number} [noiseFactor=1] - Noise factor
   */
  predict(ts, noiseFactor = 1) {
    if (!this.kf) throw new Error("No KF instance");
    if (!Number.isFinite(ts) || !Number.isFinite(noiseFactor)) {
      throw new TypeError("ts and noiseFactor must be finite numbers");
    }
    const rawTimestep = (ts - this.correctTS) / 1000;
    const timestep = rawTimestep > 0 ? rawTimestep : 0;
    const predicted = this.kf.predict({
      previousCorrected: this.previousCorrected,
      timestep,
      noiseFactor,
    });
    this.state = predicted;
    this.stateTS = Math.max(ts, this.correctTS);
  }

  /**
   * @param {number} value
   * @param {number} ts
   */
  correct(value, ts) {
    if (!this.kf) throw new Error("No KF instance");
    if (!this.state) throw new Error("No State");
    if (!Number.isFinite(value) || !Number.isFinite(ts)) {
      throw new TypeError("value and ts must be finite numbers");
    }

    /** aka H ([[1,0]] here) */
    const stateProjection = this.kf.getValue(
      this.kf.observation.stateProjection
    );
    const predictedObservation = this.kf.getPredictedObservation({
      stateProjection,
      opts: { predicted: this.state },
    });

    /** ν (nu) */
    const innovation = sub([[value]], predictedObservation);
    const nu = innovation[0][0];

    const R = this.kf.observation.covariance; // call this.kf.getValue() if needed

    /** S = H · P · Hᵀ + R — could be simplified to P + R  for 1D case */
    const S = add(
      matMul(
        matMul(stateProjection, this.state.covariance),
        transpose(stateProjection)
      ),
      R
    );

    const log2Pi = Math.log(2 * Math.PI);
    const sVal = S[0][0];
    const S_inv = Number.isFinite(sVal) && sVal > Number.MIN_VALUE ? invert(S) : null;

    let corrected;
    if (!S_inv) {
      // Singular innovation covariance (e.g., pwrStdev === 0 and P[0][0] === 0)
      this.innovation = nu;
      this.nis = nu === 0 ? 0 : Number.POSITIVE_INFINITY;
      this.logL =
        nu === 0
          ? -0.5 * (log2Pi + Math.log(Number.MIN_VALUE))
          : Number.NEGATIVE_INFINITY;
      corrected = new StateType({
        mean: this.state.mean,
        covariance: this.state.covariance,
        index: (this.state.index ?? 0) + 1,
      });
    } else {
      /**
       * NIS = innovationᵀ·inv(S)·innovation — aka squared [Mahalanobis](https://en.wikipedia.org/wiki/Mahalanobis_distance)
       * Could be simplified to ν² / S for 1D case
       */
      const nis = matMul(matMul(transpose(innovation), S_inv), innovation)[0][0];
      const detS = Math.max(sVal, Number.MIN_VALUE); // 1x1 matrix
      const logDetS = Math.log(detS);
      const logLikelihood = -0.5 * (nis + 1 * log2Pi + logDetS);

      this.innovation = nu;
      this.logL = Number.isFinite(logLikelihood)
        ? logLikelihood
        : Number.NEGATIVE_INFINITY;
      this.nis = Number.isFinite(nis) ? nis : 0;

      corrected = this.kf.correct({
        predicted: this.state,
        observation: [value],
      });
    }

    const effectiveTs = Math.max(ts, this.correctTS);
    this.previousCorrected = corrected;
    this.correctTS = effectiveTs;
    this.state = corrected;
    this.stateTS = effectiveTs;
  }

  /** @return {[number | null, number | null]} */
  mean() {
    if (this.state === null) return [null, null];
    return [this.state.mean[0][0], this.state.mean[1][0]];
  }
}

module.exports = {
  KalmanFilter,
};
