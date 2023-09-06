// @ts-check

const { KalmanFilter } = require("./monitor/kalman-filter");

const kf = new KalmanFilter(0.05, 20);

console.log(JSON.stringify(kf, null, 2));

console.log(1);

const vv = [
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];
const kf1 = new KalmanFilter(0.05, 20);

vv.forEach((v) => {
  kf1.push(v);
  const std = kf1.state ? kf1.state.covariance[0][0] ** 0.5 : 0;
  console.log(
    kf1.mean().map((v) => (v || 0).toFixed(5)),
    kf1.K.map((v) => v.toFixed(5)),
    std
  );
});

console.log(2);

const vv2 = [
  // ...new Array(10000).fill(0),
  ...[
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3,
    2, 1,
  ].map(v => v*20),
];
const kf2 = new KalmanFilter(0.05, 20);

vv2.forEach((v) => {
  kf2.push(v);
  console.log(
    kf2.mean().map((v) => (v || 0).toFixed(5)),
    kf2.K.map((v) => v.toFixed(5))
  );
});

console.log(3);

const vv3 = [
  2.01, 2.0, 2.02, 1.99, 2.0, 1.98, 2.01, 2.0, 2.02, 1.98, 2.0, 2.01, 2.0, 2.02,
  1.99, 2.0, 1.98, 2.01, 2.0, 2.02, 1.96, 2.0, 2.01, 2.0, 2.02, 1.99, 2.0, 1.98,
  2.01, 2.0, 2.02, 1.98, 2.0, 2.01, 2.0, 2.02, 1.99, 2.0, 1.98, 2.01, 2.0, 2.02,
  1.98, 2.0, 2.01, 2.0, 2.02, 1.99, 2.0, 1.98, 2.01, 2.0, 2.02, 1.98, 2.0,
];
const kf3 = new KalmanFilter(0.05, 20);
// const kf3 = kf2;

// kf3.resetCovariance();
// console.log(kf3.value.covariance);
console.log(kf3.state);
console.log(kf3.kf?.getPredictedCovariance({ previousCorrected: kf3.state }));

// return;

vv3.forEach((v) => {
  kf3.push(v);
  const [val, spd] = kf3.mean();
  const std1 = kf3.stddev() || 0;
  const std2 = kf3.state ? 3600 * kf3.state.covariance[1][1] ** 0.5 : 0;
  console.log(
    [(val || 0).toFixed(2), (3600 * (spd || 0)).toFixed(2)],
    kf3.K.map((v) => v.toFixed(5)),
    std1.toFixed(5),
    std2.toFixed(5)
  );
});

console.log(kf3.kf?.asymptoticStateCovariance());
console.log(kf3.kf?.asymptoticGain());
