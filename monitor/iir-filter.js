// @ts-check

class IIRFilter {
  /**
   * Number of data points
   */
  n = 0;
  /**
   * Mean over maxCount samples (actually 95% of mean)
   * @type {number | null}
   */
  filteredValue = null;
  /**
   * Average of squared differences from the mean
   */
  variance = 0;

  constructor(maxCount = 10) {
    this.maxCount = maxCount;
    this.reset();
  }

  reset() {
    this.n = 0;
    this.filteredValue = null;
    this.variance = 0;
  }

  // Add a new data point
  /** @param {number} value */
  push(value) {
    this.n += 1;
    if (this.filteredValue === null) {
      this.filteredValue = value;
      return;
    }
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

  count() {
    return Math.min(this.n, this.maxCount);
  }

  /** Approximate delay, in sample count */
  delay() {
    return (this.count() - 1) / 3;
  }
}

module.exports = {
  IIRFilter,
};
