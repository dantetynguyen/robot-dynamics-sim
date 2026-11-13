// lie.js — the SO(3)/SE(3) toolbox of screw theory.
//
// Function names follow the operations in Modern Robotics (MR) so each one can
// be looked up directly; symbols follow the course notes (ŝ for the unit axis,
// Ad_T without brackets). Twists are V = (ω, v), wrenches F = (τ, f):
// the rotational part is always on top.

import { cross, dot, eye, matMul, norm, vecScale, zeros } from './linalg.js';

// [ω] : 3-vector -> 3x3 skew-symmetric matrix, so that [ω] p = ω × p.
// (MR Eq. 3.30; MR code library "VecToso3")
export function skew(w) {
  return [
    [0, -w[2], w[1]],
    [w[2], 0, -w[0]],
    [-w[1], w[0], 0],
  ];
}

// Inverse of skew().
export function unskew(W) {
  return [W[2][1], W[0][2], W[1][0]];
}

// [V] : twist (ω, v) -> 4x4 matrix in se(3)  [[ω], v; 0, 0].
// (MR Eq. 3.71 / "VecTose3")
export function bracket6(V) {
  return [
    [0, -V[2], V[1], V[3]],
    [V[2], 0, -V[0], V[4]],
    [-V[1], V[0], 0, V[5]],
    [0, 0, 0, 0],
  ];
}

// Build T from a rotation matrix and a position. ("RpToTrans")
export function rpToTrans(R, p) {
  return [
    [R[0][0], R[0][1], R[0][2], p[0]],
    [R[1][0], R[1][1], R[1][2], p[1]],
    [R[2][0], R[2][1], R[2][2], p[2]],
    [0, 0, 0, 1],
  ];
}

// Split T into its rotation matrix and position. ("TransToRp")
export function transToRp(T) {
  return {
    R: [
      [T[0][0], T[0][1], T[0][2]],
      [T[1][0], T[1][1], T[1][2]],
      [T[2][0], T[2][1], T[2][2]],
    ],
    p: [T[0][3], T[1][3], T[2][3]],
  };
}

// T⁻¹ = [Rᵀ, -Rᵀp; 0, 1]: read off the structure instead of inverting
// numerically. (MR Eq. 3.64 / "TransInv")
export function transInv(T) {
  const r00 = T[0][0], r01 = T[0][1], r02 = T[0][2];
  const r10 = T[1][0], r11 = T[1][1], r12 = T[1][2];
  const r20 = T[2][0], r21 = T[2][1], r22 = T[2][2];
  const px = T[0][3], py = T[1][3], pz = T[2][3];
  return [
    [r00, r10, r20, -(r00 * px + r10 * py + r20 * pz)],
    [r01, r11, r21, -(r01 * px + r11 * py + r21 * pz)],
    [r02, r12, r22, -(r02 * px + r12 * py + r22 * pz)],
    [0, 0, 0, 1],
  ];
}

// e^{[ŝ]θ}: Rodrigues' formula for a UNIT axis ŝ.
//   R = I + sinθ [ŝ] + (1 − cosθ)[ŝ]²          (MR Eq. 3.51 / "MatrixExp3")
export function matrixExp3(sHat, theta) {
  const [x, y, z] = sHat;
  const s = Math.sin(theta), c = 1 - Math.cos(theta);
  return [
    [1 - c * (y * y + z * z), -s * z + c * x * y, s * y + c * x * z],
    [s * z + c * x * y, 1 - c * (x * x + z * z), -s * x + c * y * z],
    [-s * y + c * x * z, s * x + c * y * z, 1 - c * (x * x + y * y)],
  ];
}

// e^{[S]θ}: exponential of a screw axis S = (ω, v) times a joint value θ.
//   ‖ω‖ = 1:  R = Rodrigues(ω, θ),
//             p = (Iθ + (1 − cosθ)[ω] + (θ − sinθ)[ω]²) v      (MR Eq. 3.88)
//   ω = 0  :  R = I, p = vθ   (prismatic)
// A screw axis whose ω is neither unit nor zero is rescaled first, so a raw
// 6-vector typed by the user still gives a valid transform. ("MatrixExp6")
export function matrixExp6(S, theta) {
  let w = [S[0], S[1], S[2]];
  let v = [S[3], S[4], S[5]];
  const wn = norm(w);
  if (wn < 1e-12) {
    return rpToTrans(eye(3), vecScale(v, theta));
  }
  if (Math.abs(wn - 1) > 1e-12) {
    w = vecScale(w, 1 / wn);
    v = vecScale(v, 1 / wn);
    theta = theta * wn;
  }
  const R = matrixExp3(w, theta);
  const s = Math.sin(theta), c = 1 - Math.cos(theta);
  // p = θ v + (1 − cosθ) ω×v + (θ − sinθ) ω×(ω×v)
  const wv = cross(w, v);
  const wwv = cross(w, wv);
  const k = theta - s;
  const p = [
    theta * v[0] + c * wv[0] + k * wwv[0],
    theta * v[1] + c * wv[1] + k * wwv[1],
    theta * v[2] + c * wv[2] + k * wwv[2],
  ];
  return rpToTrans(R, p);
}

// Ad_T : the 6x6 "big adjoint" of T = (R, p),
//   Ad_T = [ R      0 ]
//          [ [p]R   R ]
// It moves a twist between frames: V_a = Ad_{T_ab} V_b.   (MR Def. 3.20)
export function adjoint(T) {
  const A = zeros(6, 6);
  const px = T[0][3], py = T[1][3], pz = T[2][3];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      const r = T[i][j];
      A[i][j] = r;
      A[i + 3][j + 3] = r;
    }
  }
  for (let j = 0; j < 3; j++) {
    const r0 = T[0][j], r1 = T[1][j], r2 = T[2][j];
    A[3][j] = -pz * r1 + py * r2;
    A[4][j] = pz * r0 - px * r2;
    A[5][j] = -py * r0 + px * r1;
  }
  return A;
}

// ad_V : the 6x6 "little adjoint" (Lie bracket) of a twist V = (ω, v),
//   ad_V = [ [ω]   0  ]
//          [ [v]  [ω] ]
// It is what the time derivative of Ad_T produces, and it carries the
// Coriolis/centripetal terms in the dynamics.   (MR Eq. 8.38)
export function ad(V) {
  const wx = V[0], wy = V[1], wz = V[2], vx = V[3], vy = V[4], vz = V[5];
  return [
    [0, -wz, wy, 0, 0, 0],
    [wz, 0, -wx, 0, 0, 0],
    [-wy, wx, 0, 0, 0, 0],
    [0, -vz, vy, 0, -wz, wy],
    [vz, 0, -vx, wz, 0, -wx],
    [-vy, vx, 0, -wy, wx, 0],
  ];
}

// Screw axis from its geometry: unit direction ŝ, a point q on the axis and
// the pitch h (0 for a revolute joint).
//   S = (ŝ, −ŝ×q + hŝ)      revolute / helical
//   S = (0, ŝ)              prismatic (h = Infinity)
// Moving q along the axis leaves S unchanged, which is a handy self-check.
export function screwAxis(sHat, q, h = 0) {
  const s = vecScale(sHat, 1 / norm(sHat));
  if (!Number.isFinite(h)) return [0, 0, 0, s[0], s[1], s[2]];
  const v = cross(q, s); // = −ŝ×q
  return [s[0], s[1], s[2], v[0] + h * s[0], v[1] + h * s[1], v[2] + h * s[2]];
}

// Recover (ŝ, q, h) from a screw axis; q is the point on the axis closest to
// the origin.
export function screwGeometry(S) {
  const w = [S[0], S[1], S[2]], v = [S[3], S[4], S[5]];
  const wn = norm(w);
  if (wn < 1e-12) {
    return { sHat: vecScale(v, 1 / (norm(v) || 1)), q: [0, 0, 0], h: Infinity };
  }
  const sHat = vecScale(w, 1 / wn);
  const vv = vecScale(v, 1 / wn);
  const h = dot(sHat, vv);
  const q = cross(sHat, vv); // ŝ × (−ŝ×q + hŝ) = q − (q·ŝ)ŝ
  return { sHat, q, h };
}

// Rotation matrices about the coordinate axes (used for building home poses).
export function rotX(a) { return matrixExp3([1, 0, 0], a); }
export function rotY(a) { return matrixExp3([0, 1, 0], a); }
export function rotZ(a) { return matrixExp3([0, 0, 1], a); }

// Product of several 4x4 transforms.
export function compose(...Ts) {
  return Ts.reduce((acc, T) => matMul(acc, T));
}
