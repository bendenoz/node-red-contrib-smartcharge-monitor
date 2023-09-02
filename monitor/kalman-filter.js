// @ts-check

const { KalmanFilter: KF } = require("kalman-filter");
const State = require("kalman-filter/lib/state");

class KalmanFilter {
  /** @type {KF} */
  kf;

  /** @type {State | null} */
  value = null;

  /**
   * @param {number} stdev process std dev
   * @param {number} timestep sample interval, in seconds
   */
  constructor(stdev, timestep) {
    this.kf = new KF({
      observation: {
        dimension: 1,
        covariance: [stdev], // diag if not a matrix
      },
      dynamic: {
        // name: "constant-speed",
        dimension: 2,
        init: {
          mean: [[0], [0]],
          covariance: [
            [1e8, 0],
            [0, 1e8],
          ],
          index: -1, // ?
        },
        transition: [
          [1, timestep],
          [0, 1],
        ],
        covariance: [stdev / 50, stdev / (timestep * 1e6)], // TODO - try stdev / 5 or 50 ?
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
    return 0;
  }

  /** Approximate delay, in sample count */
  delay() {
    // TODO - this.K[1][0]
    return 0;
  }

  // Get the current standard deviation
  stddev() {
    return 0;
  }
}

module.exports = {
  KalmanFilter,
};
