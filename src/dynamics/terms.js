// terms.js — the configuration-space equation of motion and its pieces,
//
//   τ = M(θ) θ̈ + c(θ, θ̇) + g(θ) + Σ_i D_i J_sᵀ F^ext_{0,i}      (Topic 13 p. 3)
//
// with h = c + g. Every piece comes from calls to the same RNE routine
// (Topic 13 pp. 4–5; MR §8.5), plus an independent kinetic-energy route to
// M(θ) for cross-checking (Topic 13 p. 5; MR Eq. 8.57).
//
// DH bridge: this is the D(q)q̈ + C(q,q̇)q̇ + g(q) = τ form of DH-based
// textbooks; M plays the role of D, and c is C(q,q̇)q̇ already multiplied out.

import {
  dot, fromColumns, matMul, matTVec, matVec, solve, transpose, zeros, zerosVec,
} from '../math/linalg.js';
import { adjoint, transInv } from '../math/lie.js';
import { jacobianSpace } from './kinematics.js';
import { linkKinematics, rne } from './rne.js';

// M(θ), column by column: with g = 0, θ̇ = 0 and no external wrench, RNE
// returns M(θ) θ̈, so θ̈ = e_j picks out column j.   (Topic 13 pp. 4–5; MR §8.5)
export function massMatrix(robot, kin, eqs) {
  const N = robot.N;
  const zero = zerosVec(N);
  const cols = [];
  for (let j = 0; j < N; j++) {
    const unit = zerosVec(N);
    unit[j] = 1;
    cols.push(rne(robot, kin, zero, unit, { gravity: null, eqs }).tau);
  }
  return fromColumns(cols);
}

// M(θ) from kinetic energy, K = ½ θ̇ᵀ M θ̇ = ½ Σ V_iᵀ G_i V_i:
//   M = Σ_i D_i J_sᵀ Ad_{T_0i⁻¹}ᵀ G_i Ad_{T_0i⁻¹} J_s D_i       (Topic 13 p. 5)
// D_i keeps only the first i columns of J_s (joints after i do not move
// link i). Same matrix as MR Eq. 8.57, Σ J_ibᵀ G_i J_ib.
export function massMatrixKE(robot, theta) {
  const N = robot.N;
  const kin = linkKinematics(robot, theta);
  const Js = jacobianSpace(robot, theta);
  let Mq = zeros(N, N);
  for (let i = 0; i < N; i++) {
    // J_i = Ad_{T_0i⁻¹} J_s D_i : body Jacobian of link frame {i}
    const JsDi = Js.map((row) => row.map((v, j) => (j <= i ? v : 0)));
    const Ji = matMul(adjoint(transInv(kin.T0[i])), JsDi);
    const term = matMul(transpose(Ji), matMul((robot.Gref ?? robot.G)[i], Ji));
    Mq = Mq.map((row, r) => row.map((v, c) => v + term[r][c]));
  }
  return Mq;
}

// g(θ): the torques needed just to hold still against gravity,
//   g = RNE(θ, θ̇ = 0, θ̈ = 0) with gravity on.
export function gravityTorques(robot, kin, eqs, gravity = robot.gravity) {
  const zero = zerosVec(robot.N);
  return rne(robot, kin, zero, zero, { gravity, eqs }).tau;
}

// c(θ, θ̇): Coriolis and centripetal torques,
//   c = RNE(θ, θ̇, θ̈ = 0) with gravity off.
export function coriolisTorques(robot, kin, dtheta, eqs) {
  return rne(robot, kin, dtheta, zerosVec(robot.N), { gravity: null, eqs }).tau;
}

// Joint torques that balance external wrenches:
//   τ_ext = Σ_{i=1}^{N+1} D_i J_sᵀ F^ext_{0,i}                  (Topic 13 p. 3)
// A wrench on link i reaches joints 1…i only; a wrench on the tool frame
// (i = N+1) reaches every joint, i.e. D_{N+1} = I.
// Fext is in the notes' sense (wrench the link exerts on the world).
export function externalTorques(robot, theta, Fext) {
  const N = robot.N;
  const tau = zerosVec(N);
  if (!Fext) return tau;
  const Js = jacobianSpace(robot, theta);
  for (let i = 0; i <= N; i++) {
    if (!Fext[i]) continue;
    const full = matTVec(Js, Fext[i]); // J_sᵀ F
    const reach = i === N ? N - 1 : i; // last joint this wrench is felt at
    for (let j = 0; j <= reach; j++) tau[j] += full[j];
  }
  return tau;
}

// Forward dynamics: the acceleration produced by given joint torques,
//   θ̈ = M(θ)⁻¹ ( τ − h(θ,θ̇) − τ_friction − Σ D_i J_sᵀ F^ext )  (Topic 13 p. 7; MR Eq. 8.81)
// The bias h + external term comes from ONE RNE call with θ̈ = 0 and the
// wrenches left in (the notes' "lump together" remark, Topic 13 p. 4).
//
// Returns { ddtheta, Mq, bias } or throws if M is singular (which can only
// happen when an edited equation has broken the model).
export function forwardDynamics(robot, theta, dtheta, tau, opts = {}) {
  const N = robot.N;
  const kin = opts.kin ?? linkKinematics(robot, theta);
  const Mq = massMatrix(robot, kin, opts.eqs);
  const bias = rne(robot, kin, dtheta, zerosVec(N), {
    gravity: opts.gravity === undefined ? robot.gravity : opts.gravity,
    Fext: opts.Fext,
    eqs: opts.eqs,
  }).tau;
  const rhs = new Array(N);
  for (let i = 0; i < N; i++) rhs[i] = tau[i] - bias[i] - (opts.friction ? opts.friction[i] : 0);
  const ddtheta = solve(Mq, rhs);
  if (ddtheta === null) throw new Error('mass matrix is singular');
  return { ddtheta, Mq, bias, kin };
}

// Mechanical energy.
//   K = ½ Σ V_iᵀ G_i V_i                                   (MR Eq. 8.55)
//   P = −Σ m_i g · p_i     (p_i = CoM position of link i in {0})
// With zero joint torque, no friction and no external wrench, K + P is
// constant: the first sanity check of any simulation.
// Energy is always measured with the REFERENCE equations (robot.Gref and the
// built-in twist recursion), so it stays a trustworthy yardstick even when the
// plant is running an edited, wrong equation.
export function energy(robot, theta, dtheta, kinIn) {
  const kin = kinIn ?? linkKinematics(robot, theta);
  const { V } = rne(robot, kin, dtheta, zerosVec(robot.N), { gravity: null });
  const G = robot.Gref ?? robot.G;
  let K = 0, P = 0;
  for (let i = 0; i < robot.N; i++) {
    K += 0.5 * dot(V[i], matVec(G[i], V[i]));
    const T = kin.T0[i];
    P -= robot.mass[i] * (robot.gravity[0] * T[0][3] + robot.gravity[1] * T[1][3] + robot.gravity[2] * T[2][3]);
  }
  return { K, P, E: K + P };
}

export function potentialEnergy(robot, theta) {
  return energy(robot, theta, zerosVec(robot.N)).P;
}

// All the terms at one state, for display and plots.
export function dynamicsTerms(robot, theta, dtheta, opts = {}) {
  const kin = linkKinematics(robot, theta);
  const Mq = massMatrix(robot, kin, opts.eqs);
  const c = coriolisTorques(robot, kin, dtheta, opts.eqs);
  const g = gravityTorques(robot, kin, opts.eqs, opts.gravity ?? robot.gravity);
  return { kin, Mq, c, g };
}
