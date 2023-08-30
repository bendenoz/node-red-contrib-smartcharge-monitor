class IIRFilter {
  constructor(maxCount = 10) {
    this.maxCount = maxCount;
    this.reset();
  }

  reset() {
    this.n = 0; // Number of data points
    this.filteredValue = 0; // Mean over maxCount samples (actually 95% of mean)
    this.variance = 0; // Average of squared differences from the mean
  }

  // Add a new data point
  /** @param {number} value */
  push(value) {
    this.n += 1;
    const alpha = 1 - Math.exp(-3 / Math.min(this.n, this.maxCount));
    const delta1 = value - this.filteredValue;
    this.filteredValue = (1 - alpha) * this.filteredValue + alpha * value;
    const delta2 = value - this.filteredValue;
    this.variance = (1 - alpha) * this.variance + alpha * (delta1 * delta2);
  }

  // Get the current standard deviation
  stddev() {
    return Math.sqrt(this.variance);
  }

  mean() {
    return this.filteredValue;
  }
}

module.exports = {
  IIRFilter,
};
