// wrench.js — wrench theory: screws, frames, power, statics, reciprocal
// wrenches, ellipsoids, and the motor/bearing split of a joint wrench.
//
// A wrench is F = (τ, f): moment on top, force below (Topic 10 p. 1).
// Sources: Topic 10; MR §3.4, §5.2, §5.4. The wrench screw (pitch) form,
// Poinsot's theorem and manipulator-level reciprocal screws are in the notes
// only.

import {
  cross, dot, eigSym, matMul, matTVec, norm, transpose, vecAdd, vecScale, vecSub,
} from '../math/linalg.js';
import { adjoint, transInv } from '../math/lie.js';

// ---------------------------------------------------------------------------
// Sign convention helper (CONVENTIONS.md §4).
// The engine's F^ext is the wrench a link exerts on the world (notes' sense,
// Topic 11 p. 13). A wrench APPLIED TO the link is the negative of that.
// This is the only place the conversion happens.
// ---------------------------------------------------------------------------
export function appliedToNotes(W) {
  return W.map((x) => -x);
}

// ---------------------------------------------------------------------------
// Change of frame:  F_b = Ad_{T_ab}ᵀ F_a                (Topic 10 p. 7; MR Eq. 3.95)
// Twists use the same T_ab the other way round: V_a = Ad_{T_ab} V_b.
// Reference version of the editable equation "wrenchTransform".
// ---------------------------------------------------------------------------
export function wrenchTransform(Tab, Fa) {
  return matTVec(adjoint(Tab), Fa);
}

// Express in {0} a wrench given in a frame whose pose in {0} is T0f:
//   F_0 = Ad_{T_f0}ᵀ F_f = Ad_{T_0f⁻¹}ᵀ F_f
// `transform` lets the caller pass an edited version of the frame-change rule.
export function wrenchToSpace(T0f, Ff, transform = wrenchTransform) {
  return transform(transInv(T0f), Ff);
}

// Power exchanged between a twist and a wrench in the same frame:
//   P = Vᵀ F = ωᵀτ + vᵀf                               (Topic 10 p. 3; MR Eq. 3.94)
// The number is the same in every frame. P = 0 means the pair is reciprocal.
export function power(V, F) {
  return dot(V, F);
}

// ---------------------------------------------------------------------------
// Wrench as a screw (Poinsot: a force along a line plus a couple about that
// same line).
//   finite pitch:   F = f · (−ŝ×q + hŝ, ŝ)      f = force magnitude
//   infinite pitch: F = τ · (ŝ, 0)               pure couple
//                                                        (Topic 10 pp. 2–3)
// For a wrench the pitch is moment-per-force (metres): h = 0 is a pure force,
// h = ∞ a pure couple. That is the reverse of twists, where h = 0 is a pure
// rotation.
// ---------------------------------------------------------------------------
export function wrenchToScrew(F, tol = 1e-12) {
  const tau = [F[0], F[1], F[2]];
  const f = [F[3], F[4], F[5]];
  const fn = norm(f);
  if (fn > tol) {
    const sHat = vecScale(f, 1 / fn);
    const h = dot(tau, sHat) / fn;               // couple along the line, per unit force
    const q = vecScale(cross(sHat, tau), 1 / fn); // point of the line closest to the origin
    return { kind: 'force', sHat, q, h, magnitude: fn };
  }
  const tn = norm(tau);
  if (tn > tol) {
    return { kind: 'couple', sHat: vecScale(tau, 1 / tn), q: [0, 0, 0], h: Infinity, magnitude: tn };
  }
  return { kind: 'zero', sHat: [0, 0, 1], q: [0, 0, 0], h: 0, magnitude: 0 };
}

export function screwToWrench({ sHat, q, h, magnitude }) {
  if (!Number.isFinite(h)) return [...vecScale(sHat, magnitude), 0, 0, 0];
  const moment = vecAdd(cross(q, sHat), vecScale(sHat, h)); // −ŝ×q + hŝ
  return [...vecScale(moment, magnitude), ...vecScale(sHat, magnitude)];
}

// ---------------------------------------------------------------------------
// Statics:  τ = Jᵀ F                                     (Topic 10 p. 8; MR Eq. 5.26)
// F is the wrench the tool exerts on the world, in the same frame as J
// (J_s with F_s, J_b with F_b). Reference version of editable equation
// "statics".
// ---------------------------------------------------------------------------
export function staticTorques(J, F) {
  return matTVec(J, F);
}

// ---------------------------------------------------------------------------
// Reciprocal wrenches: null(Jᵀ) = { F : Jᵀ F = 0 }       (Topic 10 p. 5; MR §5.2)
// Each such wrench does no work on any joint twist, so the structure carries
// it with zero motor torque. For a 3-joint arm away from singularities this
// space is 6 − 3 = 3-dimensional.
//
// null(Jᵀ) is the orthogonal complement of the column space of J, i.e. the
// eigenvectors of the symmetric 6x6 matrix J Jᵀ with zero eigenvalue.
// Returns { basis: [6-vectors], rank }.
// ---------------------------------------------------------------------------
export function reciprocalBasis(J, relTol = 1e-9) {
  const JJt = matMul(J, transpose(J));
  const { values, vectors } = eigSym(JJt);
  const cutoff = Math.max(values[0], 1e-300) * relTol;
  const basis = [];
  let rank = 0;
  for (let k = 0; k < values.length; k++) {
    if (values[k] > cutoff) rank++;
    else basis.push(vectors[k]);
  }
  return { basis, rank };
}

// ---------------------------------------------------------------------------
// Ellipsoids at a point (Topic 10 pp. 8–9 as corrected in class; MR §5.4).
// Jp is the 3xN position Jacobian of the point (ṗ = Jp θ̇), see
// kinematics.pointJacobian().
//
//   Manipulability ellipsoid: velocities reachable with ‖θ̇‖ = 1.
//     Axes = eigenvectors of A = Jp Jpᵀ, semi-axis lengths √λ_i.
//   Force ellipsoid: forces at the point that cost ‖τ‖ = 1.
//     Shaped by A⁻¹ = (Jp Jpᵀ)⁻¹: same axes, semi-axis lengths 1/√λ_i.
//
// The two are dual: a direction that is easy to move in is hard to push in,
// and at a singularity (some λ -> 0) the force axis grows without bound.
// Returns axes sorted by λ descending; force lengths are Infinity when λ = 0.
// ---------------------------------------------------------------------------
export function pointEllipsoids(Jp) {
  const A = matMul(Jp, transpose(Jp));
  const { values, vectors } = eigSym(A);
  const lam = values.map((v) => Math.max(v, 0));
  const manip = lam.map((l) => Math.sqrt(l));
  const force = lam.map((l) => (l > 1e-18 ? 1 / Math.sqrt(l) : Infinity));
  const sMax = manip[0], sMin = manip[manip.length - 1];
  return {
    axes: vectors,                 // unit vectors, shared by both ellipsoids
    lambda: lam,                   // eigenvalues of Jp Jpᵀ
    manipulability: manip,         // semi-axes of the velocity ellipsoid
    force,                         // semi-axes of the force ellipsoid
    // Scalar measures (MR §5.4): μ1 = longest/shortest axis, μ2 = condition
    // number of A, μ3 ∝ volume.
    mu1: sMin > 1e-12 ? sMax / sMin : Infinity,
    mu2: sMin > 1e-12 ? (sMax * sMax) / (sMin * sMin) : Infinity,
    mu3: manip.reduce((a, b) => a * b, 1),
  };
}

// ---------------------------------------------------------------------------
// Motor / bearing split of a joint wrench.
//
// F_i (from the RNE backward pass) is the whole wrench passed through joint i
// to link i, in frame {i}. The motor of a revolute joint can only apply a pure
// couple about its own axis, τ_i·(ŝ, 0) (the "applied wrench" of the joint
// table, Topic 10 p. 6). Everything else is reciprocal to the joint twist and
// is carried passively by the bearing:
//
//   F_i = F_motor + F_bearing,   F_motor = τ_i (ŝ_i, 0),   F_bearingᵀ A_i = 0,
//   τ_i = F_iᵀ A_i.
//
// `A` is the joint screw in {i}. For display the bearing load is also reduced
// to the joint axis: axial (thrust) force, radial force, and bending moment
// about the point of the axis closest to the {i} origin.
// (Revolute joints only; a prismatic joint's motor applies a pure force.)
// ---------------------------------------------------------------------------
export function splitJointWrench(F, A) {
  const sHat = [A[0], A[1], A[2]];
  const tauMotor = dot(F, A);
  const motor = [sHat[0] * tauMotor, sHat[1] * tauMotor, sHat[2] * tauMotor, 0, 0, 0];
  const bearing = vecSub(F, motor);

  // Reduce the bearing wrench to the point of the axis nearest the frame
  // origin: q = ŝ × v_A (because v_A = −ŝ × q). Moving the reference point of
  // a wrench by q changes the moment by −q × f.
  const q = cross(sHat, [A[3], A[4], A[5]]);
  const f = [bearing[3], bearing[4], bearing[5]];
  const mAtAxis = vecSub([bearing[0], bearing[1], bearing[2]], cross(q, f));
  const axialForce = dot(f, sHat);
  const radial = vecSub(f, vecScale(sHat, axialForce));
  const axialMoment = dot(mAtAxis, sHat); // zero by construction
  const bending = vecSub(mAtAxis, vecScale(sHat, axialMoment));
  return {
    tauMotor,
    motor,
    bearing,
    axialForce,
    radialForce: norm(radial),
    bendingMoment: norm(bending),
    forceMagnitude: norm(f),
    residualAxialMoment: axialMoment,
  };
}
