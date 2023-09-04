const KalmanClass = require("kalman-filter/lib/kalman-filter");
const StateType = require("kalman-filter/lib/state");

require("kalman-filter"); // must be required to init default models

class KalmanFilter {
  /** @type {KalmanClass} */
  kf;

  /** @type {StateType} */
  value = null;

  /** @type {[number, number]} */
  K = [0, 0];

  /**
   * @param {number} stdev process std dev
   * @param {number} timestep sample interval, in seconds
   */
  constructor(stdev, timestep) {
    this.kf = new KalmanClass({
      observation: {
        dimension: 1,
        // R
        covariance: [stdev ** 2], // diag if not a matrix
      },
      dynamic: {
        // name: "constant-speed",
        dimension: 2,
        init: {
          mean: [[0], [0]],
          // Initial P
          covariance: [
            [10, 0],
            [0, 10 ** -6],
          ],
          index: -1, // ?
        },
        transition: [
          [1, timestep],
          [0, 1],
        ],
        // Q
        covariance: [
          stdev ** 2 / 10 ** 1.5, // stdev ** 2 / 10 ** 1,
          10 ** -8, // stdev ** 2 / 10 ** 1.5, //  / ((30 * 20) / timestep) ** 2,
        ], // gain of 0.12856 (~20 samples) for value stabilized
      },
    });
  }

  resetCovariance() {
    // reset P0 to speed-up recovery
    this.value.covariance = this.kf.getInitState().covariance;
    // also reset velocity to 0
    this.value.mean[1] = [0];
  }

  // Add a new data point
  /** @param {number} value */
  push(value) {
    const predicted = this.kf.predict({
      previousCorrected: this.value,
    });

    this.K = /** @type {[number, number]} */ (
      this.kf.getGain({ predicted }).map(([v]) => v)
    );
    // console.log('gain', this.gain);

    const correctedState = this.kf.correct({
      predicted,
      observation: [value],
    });

    this.value = correctedState;
  }

  /** @return {number[]} */
  mean() {
    return this.value === null ? [null, null] : this.value.mean.map(([v]) => v);
  }

  count() {
    return this.value === null ? 0 : this.value.index;
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
    return this.value.covariance[0][0] ** 0.5;
  }
}

module.exports = {
  KalmanFilter,
};
