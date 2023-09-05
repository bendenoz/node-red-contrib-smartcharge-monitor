class RollingDerivate {
  size = 5;
  // http://www.statistics4u.info/fundstat_eng/cc_savgol_coeff.html
  coeffs = [-2, -1, 0, 1, 2];
  h = 10;

  /** @type {number[]} */
  buffer;

  /** @param {number} timestep */
  constructor(timestep) {
    this.buffer = [];
    this.timestep = timestep;
  }

  /** @param {number} item */
  push(item) {
    if (this.buffer.length >= this.size) {
      this.buffer.shift(); // Remove the oldest item if buffer is full
    }
    this.buffer.push(item);
  }

  getAll() {
    return this.buffer;
  }

  /** Savitzky Golay first derivative calculation */
  sg() {
    if (this.buffer.length < this.size) return 0;
    return (
      this.buffer.reduce((r, v, i) => r + this.coeffs[i] * v, 0) /
      (this.h * this.timestep)
    );
  }
}

module.exports = {
  RollingDerivate,
};
