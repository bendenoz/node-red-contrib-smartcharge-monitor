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

const pwrStdev = 0.08; // measured 0.05 - 0.07 - TODO: Make adjustable

class KalmanFilter {
  /** @type {KalmanClass | undefined} */
  kf;

  /** @type {StateType | null} */
  state = null;

  /** @type {StateType | null} */
  previousCorrected = null;

  /** @type {[number, number]} */
  K = [0, 0];

  /**
   * previous timestamp, in milliseconds
   * @type {number}
   */
  stateTS;
  /**
   * previous timestamp, in milliseconds
   * @type {number}
   */
  correctTS;

  /**
   * k noise - in units per second per square-rooted second (not units^2)
   * @type {number}
   */
  kStdev;

  /**
   * @param {number} kStdev process std dev
   */
  constructor(kStdev) {
    this.kStdev = kStdev;
  }

  /**
   * Instanciate kalman filter with value
   * @param {number} initValue
   * @param {number} initTs
   */
  init(initValue, initTs) {
    try {
      this.kf = new KalmanClass({
        observation: {
          dimension: 1,
          // R
          covariance: [[pwrStdev ** 2]],
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

            // old - was very close
            const kNoise = noiseFactor * this.kStdev * timestep ** 0.5;
            const rateNoise = Math.abs(target - pwr) * kNoise;
            const pwrNoise = (rateNoise * timestep) / 1.5; // divide by 1.5 because integration(?)
            const correl = 1; // assume full correlation
            return [
              [pwrNoise ** 2, correl * pwrNoise * kNoise],
              [correl * pwrNoise * kNoise, kNoise ** 2],
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
  }

  resetCovariance() {
    if (!this.state || !this.kf) return;
    this.state.covariance = this.kf.getInitState().covariance;
  }

  /**
   * Predict the next state
   * @param {number} ts - Timestamp in milliseconds
   * @param {number} [noiseFactor=1] - Noise factor
   */
  predict(ts, noiseFactor = 1) {
    if (!this.kf || !this.correctTS) throw new Error("No KF instance");
    const timestep = (ts - this.correctTS) / 1000;
    const predicted = this.kf.predict({
      previousCorrected: this.previousCorrected,
      timestep,
      noiseFactor,
    });
    this.state = predicted;
    this.stateTS = ts;
  }

  /**
   * @param {number} value
   * @param {number} ts
   */
  correct(value, ts) {
    if (!this.kf) throw new Error("No KF instance");
    if (!this.state) throw new Error("No State");

    // calculate mahalanobis and logLikelihood

    // Not using this.kf.observation.mahalanobis() because we need S for logL, and it looks broken
    // this.mahalanobis = this.state.mahalanobis({
    //   kf: this.kf,
    //   observation: [value],
    //   obsIndexes: [0],
    // });

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

    // calculate Normalized Innovation Squared (NIS)

    const R = this.kf.observation.covariance; // call this.kf.getValue() if needed

    /** S = H · P · Hᵀ + R — could be simplified to P + R  for 1D case */
    const S = add(
      matMul(
        matMul(stateProjection, this.state.covariance),
        transpose(stateProjection)
      ),
      R
    );

    const S_inv = invert(S);

    /**
     * NIS = innovationᵀ·inv(S)·innovation — aka squared [Mahalanobis](https://en.wikipedia.org/wiki/Mahalanobis_distance)
     * Could be simplified to ν² / S for 1D case
     */
    const nis = matMul(matMul(transpose(innovation), S_inv), innovation)[0][0];

    const detS = S[0][0]; // 1x1 matrix !
    // Compute necessary logarithms
    const logDetS = Math.log(detS); // log determinant of S
    const log2Pi = Math.log(2 * Math.PI);

    // Compute the log-likelihood using the multivariate normal PDF formula
    // logLikelihood = -0.5 * (nis + n * log(2π) + log(detS))
    const logLikelihood = -0.5 * (nis + 1 * log2Pi + logDetS);

    this.logL = logLikelihood;
    this.nis = nis;

    const corrected = this.kf.correct({
      predicted: this.state,
      observation: [value],
    });
    this.previousCorrected = corrected;
    this.correctTS = ts; // = this.stateTS ? since predict is called first
    this.state = corrected;
    this.stateTS = ts;
  }

  /** @return {[number | null, number | null]} */
  mean() {
    return this.state === null ? [null, null] : this.state.mean.map(([v]) => v);
  }

  count() {
    return this.state === null ? 0 : this.state.index;
  }

  /** Approximate delay, in sample count */
  delay() {
    const K = this.K[0][0];
    const tc = -1 / Math.log(1 - K);
    return 2 * Math.log(2) * tc; // 1.4 time constant
  }

  // Get the current standard deviation
  stddev() {
    // the first entry in the predicted covariance is value variance
    return this.state && this.state.covariance[0][0] ** 0.5;
  }
}

module.exports = {
  pwrStdev,
  KalmanFilter,
};
