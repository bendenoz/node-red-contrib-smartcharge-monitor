const KalmanClass = require("kalman-filter/lib/kalman-filter");
const StateType = require("kalman-filter/lib/state");

require("kalman-filter"); // must be required to init default models

class KalmanFilter {
  /** @type {KalmanClass} */
  kf;

  /** @type {StateType} */
  value = null;

  /**
   * @param {number} stdev process std dev
   * @param {number} timestep sample interval, in seconds
   */
  constructor(stdev, timestep) {
    this.kf = new KalmanClass({
      observation: {
        dimension: 1,
        covariance: [stdev ** 2], // diag if not a matrix
      },
      dynamic: {
        // name: "constant-speed",
        dimension: 2,
        init: {
          mean: [[0], [0]],
          covariance: [
            [1e6, 0],
            [0, 1e6],
          ],
          index: -1, // ?
        },
        transition: [
          [1, timestep],
          [0, 1],
        ],
        covariance: [stdev ** 2 / 10 ** 2, stdev ** 2 / 10 ** 5 / 300], // gain of 0.12856 (~20 samples) for value stabilized
      },
    });
  }

  // Add a new data point
  /** @param {number} value */
  push(value) {
    const predicted = this.kf.predict({
      previousCorrected: this.value,
    });

    this.K = this.kf.getGain({ predicted });
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
