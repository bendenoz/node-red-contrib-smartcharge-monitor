// @ts-check

const KF = require("kalmanjs");

class SimpleKalmanFilter {
  /** @type {ReturnType<typeof KF>} */
  kf;

  constructor(opts = { R: 0.0001, Q: 0.05 }) {
    this.kf = new KF(opts);
  }

  // Add a new data point
  /** @param {number} value */
  push(value) {
    this.kf.filter(value);
  }

  mean() {
    return this.kf.lastMeasurement();
  }

  count() {
    return 0;
  }

  /** Approximate delay, in sample count */
  delay() {
    // We retrieve (calculate) K from Kalman, it's our Alpha
    const { uncertainty, C, Q } = this.kf;
    const predCov = uncertainty(); // Kalman gain
    var K = predCov * C * (1 / (C * predCov * C + Q)); // Correction

    const count = -3 / Math.log(1 - K);
    return (count - 1) / 2;
  }

  // Get the current standard deviation
  stddev() {
    return this.kf.Q;
  }
}

module.exports = {
  SimpleKalmanFilter,
};
