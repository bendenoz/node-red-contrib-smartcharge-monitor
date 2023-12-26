const { invert, matMul } = require("simple-linalg");

const rotVectors = (/** @type {number} */ angle) => {
  return [
    [Math.cos(angle), -Math.sin(angle)],
    [Math.sin(angle), Math.cos(angle)],
  ];
};

const covarMatrix = (
  /** @type {number} */ lambda1,
  /** @type {number} */ lambda2,
  /** @type {number} */ angle
) => {
  // Creating the diagonal matrix D with the eigenvalues
  const D = [
    [lambda1 ** 2, 0],
    [0, lambda2 ** 2],
  ];

  // The matrix P formed by the eigenvectors
  const P = rotVectors(angle);

  // Calculate P inverse
  const P_inv = invert(P);

  // Reconstruct the original matrix A = PDP^-1
  const A1 = matMul(P, D);
  return matMul(A1, P_inv);
};

// console.log(covarMatrix(15, 15, (45 * Math.PI) / 180));
