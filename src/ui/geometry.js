// geometry.js — where things are in space at a given θ, for drawing.
// No three.js here: plain arrays, so the tests can use it too.

import { exponentialProducts } from '../dynamics/kinematics.js';
import { matMul } from '../math/linalg.js';
import { screwGeometry } from '../math/lie.js';

const apply = (T, p) => [
  T[0][0] * p[0] + T[0][1] * p[1] + T[0][2] * p[2] + T[0][3],
  T[1][0] * p[0] + T[1][1] * p[1] + T[1][2] * p[2] + T[1][3],
  T[2][0] * p[0] + T[2][1] * p[1] + T[2][2] * p[2] + T[2][3],
];
const rotate = (T, v) => [
  T[0][0] * v[0] + T[0][1] * v[1] + T[0][2] * v[2],
  T[1][0] * v[0] + T[1][1] * v[1] + T[1][2] * v[2],
  T[2][0] * v[0] + T[2][1] * v[1] + T[2][2] * v[2],
];

// Home position of a point on each joint axis and the axis direction.
export function homeAxes(robot) {
  return robot.S.map((S, i) => {
    const j = robot.spec.joints[i];
    if (j && j.sHat && j.q) { const n = Math.hypot(...j.sHat) || 1; return { q: j.q, sHat: j.sHat.map((x) => x / n) }; }
    const g = screwGeometry(S);
    return { q: g.q, sHat: g.sHat };
  });
}

// Everything the 3D view needs for one configuration:
//   T0           poses of the N link CoM frames and the tool frame in {0}
//   jointPoints  current position of a point on each joint axis
//   jointAxes    current direction of each joint axis
//   tool         tool-point position
// Joint i's axis is moved by the joints before it: P_{i-1} = e^{[S_1]θ_1}…e^{[S_{i-1}]θ_{i-1}}.
export function poseGeometry(robot, theta) {
  const P = exponentialProducts(robot, theta);
  const home = homeAxes(robot);
  const T0 = [];
  for (let i = 0; i < robot.N; i++) T0.push(matMul(P[i + 1], robot.Mlink[i]));
  T0.push(matMul(P[robot.N], robot.Mee));
  const Ttool = T0[robot.N];
  return {
    theta: theta.slice(),
    T0,
    jointPoints: home.map((h, i) => apply(P[i], h.q)),
    jointAxes: home.map((h, i) => rotate(P[i], h.sHat)),
    tool: [Ttool[0][3], Ttool[1][3], Ttool[2][3]],
  };
}

// The stick figure of the arm: base → joint 1 → CoM 1 → joint 2 → … → tool.
export function skeleton(geom) {
  const pts = [[0, 0, 0]];
  for (let i = 0; i < geom.jointPoints.length; i++) {
    pts.push(geom.jointPoints[i]);
    const T = geom.T0[i];
    pts.push([T[0][3], T[1][3], T[2][3]]);
  }
  pts.push(geom.tool);
  return pts;
}
