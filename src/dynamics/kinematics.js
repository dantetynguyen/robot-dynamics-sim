// kinematics.js — forward kinematics and Jacobians by the product of
// exponentials (PoE). No link frames are needed: just the home pose M and the
// screw axes S_i in the space frame.
//
// DH bridge: the pose returned by fkSpace() is the same transform a
// Denavit–Hartenberg chain would give for the same arm; PoE just reaches it
// without assigning a frame to every link.

import { fromColumns, matMul, matVec, eye, zeros } from '../math/linalg.js';
import { adjoint, matrixExp6, transInv } from '../math/lie.js';

// T(θ) = e^{[S_1]θ_1} e^{[S_2]θ_2} … e^{[S_N]θ_N} M        (MR Eq. 4.14)
export function fkSpace(robot, theta) {
  let T = eye(4);
  for (let i = 0; i < robot.N; i++) T = matMul(T, matrixExp6(robot.S[i], theta[i]));
  return matMul(T, robot.Mee);
}

// The partial products P_i = e^{[S_1]θ_1} … e^{[S_i]θ_i} for i = 0…N
// (P_0 = I). Everything else in this file is built from these.
export function exponentialProducts(robot, theta) {
  const P = [eye(4)];
  for (let i = 0; i < robot.N; i++) P.push(matMul(P[i], matrixExp6(robot.S[i], theta[i])));
  return P;
}

// Pose of every link CoM frame and of the tool frame in {0}:
//   T_{0,i}(θ) = e^{[S_1]θ_1} … e^{[S_i]θ_i} M_i          (Topic 11 p. 12)
// Returns N+1 transforms; the last one is the tool frame.
export function linkPoses(robot, theta) {
  const P = exponentialProducts(robot, theta);
  const poses = [];
  for (let i = 0; i < robot.N; i++) poses.push(matMul(P[i + 1], robot.Mlink[i]));
  poses.push(matMul(P[robot.N], robot.Mee));
  return poses;
}

// Space Jacobian, 6xN. Column i is joint i's screw axis moved to the current
// configuration by the joints before it:
//   J_s,i(θ) = Ad_{e^{[S_1]θ_1} … e^{[S_{i-1}]θ_{i-1}}} S_i   (MR Eq. 5.11)
// Column 1 is just S_1. V_s = J_s(θ) θ̇.
//
// DH bridge: for a revolute joint this column is (ẑ, −ẑ × o) with ẑ the
// current joint axis and o a point on it — the same information as the DH
// column (ẑ × (o_n − o), ẑ), but referred to the space-frame origin instead
// of the tool point.
export function jacobianSpace(robot, theta) {
  const P = exponentialProducts(robot, theta);
  const cols = [];
  for (let i = 0; i < robot.N; i++) cols.push(matVec(adjoint(P[i]), robot.S[i]));
  return fromColumns(cols);
}

// Body Jacobian: the same twist expressed in the tool frame,
//   J_b(θ) = Ad_{T(θ)⁻¹} J_s(θ)                              (MR Eq. 5.22)
export function jacobianBody(robot, theta) {
  const T = fkSpace(robot, theta);
  return matMul(adjoint(transInv(T)), jacobianSpace(robot, theta));
}

// Body Jacobian built directly from body screw axes B_i = Ad_{M⁻¹} S_i:
//   J_b,i(θ) = Ad_{e^{-[B_N]θ_N} … e^{-[B_{i+1}]θ_{i+1}}} B_i   (MR Eq. 5.18)
// Kept as an independent route; the tests check it against jacobianBody().
export function jacobianBodyDirect(robot, theta) {
  const AdMinv = adjoint(transInv(robot.Mee));
  const B = robot.S.map((S) => matVec(AdMinv, S));
  const cols = new Array(robot.N);
  let T = eye(4);
  for (let i = robot.N - 1; i >= 0; i--) {
    cols[i] = matVec(adjoint(T), B[i]);
    T = matMul(T, matrixExp6(B[i], -theta[i]));
  }
  return fromColumns(cols);
}

// Linear velocity of a point p (given in {0}) that rides on the moving body:
//   ṗ = v_s + ω_s × p = [ −[p]  I ] V_s
// Applying that to every column of J_s gives the 3xN "position Jacobian" of
// the point, ṗ = J_p θ̇. This is what the ellipsoids at the tool point use.
//
// DH bridge: J_p is exactly the J_v block of the DH geometric Jacobian.
export function pointJacobian(Js, p) {
  const N = Js[0].length;
  const Jp = zeros(3, N);
  for (let j = 0; j < N; j++) {
    const wx = Js[0][j], wy = Js[1][j], wz = Js[2][j];
    Jp[0][j] = Js[3][j] + (wy * p[2] - wz * p[1]);
    Jp[1][j] = Js[4][j] + (wz * p[0] - wx * p[2]);
    Jp[2][j] = Js[5][j] + (wx * p[1] - wy * p[0]);
  }
  return Jp;
}
