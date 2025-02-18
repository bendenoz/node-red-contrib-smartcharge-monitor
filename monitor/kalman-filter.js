"use strict";

const KalmanClass = require("kalman-filter/lib/kalman-filter");
const StateType = require("kalman-filter/lib/state");
const { performance } = require("perf_hooks"); // not needed for node > ??

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
   */
  init(initValue, initTs = performance.now()) {
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
            covariance: [100 ** 2, 1e3 ** 2],
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
            previousCorrected: { mean: [[pwr]], },
            timestep,
          }) => {
            const target = 0;
            const kNoise = this.kStdev * timestep ** .5;
            const rateNoise = (target - pwr) * kNoise;
            const pwrNoise = rateNoise * timestep;
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

  /** Returns timestep in seconds, used by correct */
  predict(ts = performance.now()) {
    if (!this.kf || !this.correctTS) throw new Error("No KF instance");
    const timestep = (ts - this.correctTS) / 1000;
    const predicted = this.kf.predict({
      previousCorrected: this.state,
      timestep,
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
    const corrected = this.kf.correct({
      predicted: this.state,
      observation: [value],
    });
    this.previousCorrected = corrected;
    this.correctTS = ts;
    this.state = corrected;
    this.stateTS = ts;
  }

  /** @return {[number | null, number | null]} */
  mean() {
    return this.state === null
      ? [null, null]
      : this.state.mean.map(([v]) => v);
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
