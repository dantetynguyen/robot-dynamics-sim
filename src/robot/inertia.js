// inertia.js — rotational inertia of simple shapes and the spatial inertia G.
//
// All inertias are about the centre of mass, in the link's own CoM frame.
// Shape formulas: uniform-density box, solid cylinder, solid ellipsoid
// (Topic 11 p. 5; MR Fig. 8.5).

import { diag, eigSym, zeros } from '../math/linalg.js';

// Returns the 3x3 rotational inertia matrix for a shape description.
//   { type: 'cylinder', r, h }      axis along the local z axis
//   { type: 'box', lx, ly, lz }     edge lengths along local x, y, z
//   { type: 'ellipsoid', a, b, c }  semi-axes along local x, y, z
//   { type: 'matrix', I }           raw 3x3 (must be symmetric positive definite)
export function shapeInertia(mass, shape) {
  switch (shape.type) {
    case 'cylinder': {
      const { r, h } = shape;
      const Ixx = (mass / 12) * (3 * r * r + h * h);
      return diag([Ixx, Ixx, (mass / 2) * r * r]);
    }
    case 'box': {
      const { lx, ly, lz } = shape;
      return diag([
        (mass / 12) * (ly * ly + lz * lz),
        (mass / 12) * (lx * lx + lz * lz),
        (mass / 12) * (lx * lx + ly * ly),
      ]);
    }
    case 'ellipsoid': {
      const { a, b, c } = shape;
      return diag([
        (mass / 5) * (b * b + c * c),
        (mass / 5) * (a * a + c * c),
        (mass / 5) * (a * a + b * b),
      ]);
    }
    case 'matrix':
      return shape.I.map((row) => row.slice());
    default:
      throw new Error(`Unknown inertia shape "${shape.type}"`);
  }
}

// Spatial inertia at the centre of mass:
//   G = [ I   0  ]
//       [ 0   m·1 ]                         (Topic 11 p. 5; MR Eq. 8.32)
// This is the reference version of the editable equation "inertia".
export function spatialInertia(I, mass) {
  const G = zeros(6, 6);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) G[i][j] = I[i][j];
    G[i + 3][i + 3] = mass;
  }
  return G;
}

// A physically valid rotational inertia is symmetric positive definite and
// its principal moments satisfy the triangle inequality (each one is at most
// the sum of the other two). Returns a list of problems (empty = fine).
export function inertiaProblems(I) {
  const problems = [];
  for (let i = 0; i < 3; i++) {
    for (let j = i + 1; j < 3; j++) {
      if (Math.abs(I[i][j] - I[j][i]) > 1e-12) problems.push('inertia matrix is not symmetric');
    }
  }
  const lam = eigSym(I).values; // descending
  if (!(lam[2] > 0)) problems.push('inertia matrix is not positive definite');
  else if (lam[0] > lam[1] + lam[2] + 1e-12) problems.push('principal moments violate the triangle inequality');
  return [...new Set(problems)];
}
