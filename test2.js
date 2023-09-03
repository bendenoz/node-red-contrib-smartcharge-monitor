const { getCovariance, KalmanFilter } = require("kalman-filter");
const State = require("kalman-filter/lib/state");

// Ground truth values in the dynamic model hidden state
const groundTruthStates = [
  // here this is (x, vx)
  [
    [0, 1.1],
    [1.1, 1],
    [2.1, 0.9],
    [3, 1],
    [4, 1.2],
  ], // example 1
  [
    [8, 1.1],
    [9.1, 1],
    [10.1, 0.9],
    [11, 1],
    [12, 1.2],
  ], // example 2
];

// Observations of this values
const measures = [
  // here this is x only
  [[0.1], [1.3], [2.4], [2.6], [3.8]], // example 1
  [[8.1], [9.3], [10.4], [10.6], [11.8]], // example 2
];

const kFilter = new KalmanFilter({
  observation: {
    name: "sensor",
    sensorDimension: 1,
  },
  dynamic: {
    name: "constant-speed",
  },
});

const observationCovariance = getCovariance({
  measures: [].concat(...measures), // measures.reduce((a, b) => a.concat(b)),
  averages: [].concat(...groundTruthStates).map(([a]) => [a]),
});

console.log("observation covariance", observationCovariance);

const reversed = getCovariance({
  averages: [].concat(...measures), // measures.reduce((a, b) => a.concat(b)),
  measures: [].concat(...groundTruthStates).map(([a]) => [a]),
});

console.log("reversed covariance", reversed);

const gt = groundTruthStates[0];
const predicted = [];
gt.slice(0, -1).forEach((truth) => {
  const s = new State({
    mean: truth.map((v) => [v]),
    covariance: [
      [1, 0],
      [0, 1],
    ],
    index: 0,
  });
  predicted.push(
    kFilter.predict({ previousCorrected: s }).mean.map(([v]) => v)
  );
});

console.log({
  measures: gt.slice(1),
  averages: predicted,
});

const dynamicCovariance = getCovariance({
  measures: gt.slice(1),
  averages: predicted,
});

console.log("dynamic covariance", dynamicCovariance);

// const observationCovariance = getCovariance({
//   measures: measures.reduce((a, b) => a.concat(b)),
//   averages: groundTruthStates.map((a) => a[0]).reduce((a, b) => a.concat(b)),
// });
