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
  const std = kf1.value.covariance[0][0] ** 0.5;
  console.log(
    kf1.mean().map((v) => v.toFixed(5)),
    kf1.delay().toFixed(5),
    std
  );
});

console.log(2);

const vv2 = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
const kf2 = new KalmanFilter(0.05, 20);

vv2.forEach((v) => {
  kf2.push(v);
  console.log(kf2.mean());
});

console.log(3);

const vv3 = [
  2.01, 2.0, 2.02, 1.99, 2.0, 1.98, 2.01, 2.0, 2.02, 1.98, 2.0, 2.01, 2.0, 2.02,
  1.99, 2.0, 1.98, 2.01, 2.0, 2.02, 1.96, 2.0, 2.01, 2.0, 2.02, 1.99, 2.0, 1.98,
  2.01, 2.0, 2.02, 1.98, 2.0, 2.01, 2.0, 2.02, 1.99, 2.0, 1.98, 2.01, 2.0, 2.02,
  1.98, 2.0, 2.01, 2.0, 2.02, 1.99, 2.0, 1.98, 2.01, 2.0, 2.02, 1.98, 2.0,
];
const kf3 = new KalmanFilter(0.05, 20);

vv3.forEach((v) => {
  kf3.push(v);
  const [val, spd] = kf3.mean();
  const std1 = kf3.stddev();
  const std2 = 3600 * kf3.value.covariance[1][1] ** 0.5;
  console.log(
    [val, 3600 * spd],
    kf3.delay().toFixed(5),
    std1.toFixed(5),
    std2.toFixed(5)
  );
});
