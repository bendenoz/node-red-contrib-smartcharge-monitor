"use strict";

const KalmanClass = require("kalman-filter/lib/kalman-filter");
const StateType = require("kalman-filter/lib/state");

require("kalman-filter"); // must be required to init default models

class KalmanFilter {
  /** @type {KalmanClass | undefined} */
  kf;

  /** @type {StateType | null} */
  state = null;

  /** @type {[number, number]} */
  K = [0, 0];

  /**
   * sample interval, in seconds
   * @type {number}
   */
  timestep;

  /**
   * process std dev
   * @type {number}
   */
  stdev;

  /**
   * speed average factor
   * @type {number}
   */
  velAvg;

  /**
   * @param {number} stdev process std dev
   * @param {number} timestep sample interval, in seconds
   */
  constructor(stdev, timestep, velAvg = 160) {
    this.timestep = timestep;
    this.stdev = stdev;
    this.velAvg = velAvg;
  }

  /**
   * Instanciate kalman filter with value
   * @param {number} initValue
   */
  init(initValue) {
    this.kf = new KalmanClass({
      observation: {
        dimension: 1,
        // R
        covariance: [this.stdev ** 2], // diag if not a matrix
      },
      dynamic: {
        // name: "constant-speed",
        dimension: 2,
        init: {
          mean: [[initValue], [0]],
          // Initial P
          covariance: [
            [10, 0],
            [0, 10 ** -4.5],
          ],
          index: -1,
        },
        transition: [
          [1, this.timestep],
          [0, 1],
        ],
        // Q
        covariance: [
          // (this.stdev / ((8 * 10) / this.timestep)) ** 2, // stdev ** 2 / 10 ** 1, // best i test  / 3 - 5
          // (this.stdev / ((this.velAvg * 10) / this.timestep)) ** 2, // we want stable velocity in w/h // best i test / 200 - 500 - 3 / 20 + 3 / 2000 ?
          (this.stdev ** 2 * this.timestep) / this.velAvg,
          ((this.stdev ** 2) ** 2 * this.timestep) / this.velAvg
        ], // gain of 0.12856 (~20 samples) for value stabilized
      },
    });
  }

  /** @param {number} initValue */
  resetCovariance(initValue) {
    if (!this.state || !this.kf) return;
    // reset P0 to speed-up recovery
    this.state.covariance = this.kf.getInitState().covariance;
    this.state.index = -1;
    this.state.mean[0] = [initValue]; // reset init value
    this.state.mean[1] = [0]; // also reset velocity to 0
    // TODO - just reset this.state and this.kf ?
    // TODO - replay last N samples
  }

  // Add a new data point
  /** @param {number} value */
  push(value) {
    if (!this.kf) this.init(value);
    if (!this.kf) throw new Error("No KF instance");

    const predicted = this.kf.predict({
      previousCorrected: this.state,
    });

    this.K = /** @type {[number, number]} */ (
      this.kf.getGain({ predicted }).map(([v]) => v)
    );
    // console.log('gain', this.gain);

    const correctedState = this.kf.correct({
      predicted,
      observation: [value],
    });

    this.state = correctedState;
  }

  /** @return {(number | null)[]} */
  mean() {
    return this.state === null ? [null, null] : this.state.mean.map(([v]) => v);
  }

  count() {
    return this.state === null ? 0 : this.state.index;
  }

  /** Approximate delay, in sample count */
  delay() {
    const K = this.K[0][0];
    const count = -3 / Math.log(1 - K);
    return (count - 1) / 3;
  }

  // Get the current standard deviation
  stddev() {
    // the first entry in the predicted covariance is value variance
    return this.state && this.state.covariance[0][0] ** 0.5;
  }
}

module.exports = {
  KalmanFilter,
};
