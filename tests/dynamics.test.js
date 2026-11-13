// Dynamics acceptance checks: M, c, g against the independent Python
// reference and against three physics identities (kinetic energy, potential
// energy gradient, Christoffel symbols).
import test from 'node:test';
import { linkKinematics, rne, inverseDynamics, referenceEquations } from '../src/dynamics/rne.js';
import {
  coriolisTorques, externalTorques, forwardDynamics, gravityTorques, massMatrix, massMatrixKE, potentialEnergy,
} from '../src/dynamics/terms.js';
import { asymmetry, cross, eigSym, matVec, zerosVec } from '../src/math/linalg.js';
import { spatialInertia } from '../src/robot/inertia.js';
import { defaultRobot, loadFixtures, maxDiff, record, rng, useResultsFile } from './helpers.js';

useResultsFile(import.meta.url);
const sets = loadFixtures();
const M = (robot, theta) => massMatrix(robot, linkKinematics(robot, theta));

test('M, c, g match the Python reference at 100 random states', () => {
  for (const { label, robot, states } of sets) {
    let wM = 0, wc = 0, wg = 0;
    for (const s of states) {
      const kin = linkKinematics(robot, s.theta);
      wM = Math.max(wM, maxDiff(massMatrix(robot, kin), s.M));
      wc = Math.max(wc, maxDiff(coriolisTorques(robot, kin, s.dtheta), s.c));
      wg = Math.max(wg, maxDiff(gravityTorques(robot, kin), s.g));
    }
    record(`M(θ) vs Python reference — ${label}`, wM, 1e-9);
    record(`c(θ,θ̇) vs Python reference — ${label}`, wc, 1e-9);
    record(`g(θ) vs Python reference — ${label}`, wg, 1e-9);
  }
});

test('M from RNE equals M from kinetic energy', () => {
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states) worst = Math.max(worst, maxDiff(M(robot, s.theta), massMatrixKE(robot, s.theta)));
    record(`M: RNE columns vs kinetic-energy formula — ${label}`, worst, 1e-9);
  }
});

test('M is symmetric positive definite at every sampled state', () => {
  for (const { label, robot, states } of sets) {
    let minEig = Infinity, asym = 0;
    for (const s of states) {
      const Mq = M(robot, s.theta);
      asym = Math.max(asym, asymmetry(Mq));
      minEig = Math.min(minEig, eigSym(Mq).values[robot.N - 1]);
    }
    record(`M symmetry |M − Mᵀ| — ${label}`, asym, 1e-12);
    record(`M smallest eigenvalue — ${label}`, minEig, 1e-6, { dir: '>=', unit: 'kg·m²' });
  }
});

test('g equals the gradient of potential energy', () => {
  const h = 1e-6;
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states) {
      const g = gravityTorques(robot, linkKinematics(robot, s.theta));
      for (let j = 0; j < robot.N; j++) {
        const tp = s.theta.slice(), tm = s.theta.slice();
        tp[j] += h; tm[j] -= h;
        const dP = (potentialEnergy(robot, tp) - potentialEnergy(robot, tm)) / (2 * h);
        worst = Math.max(worst, Math.abs(dP - g[j]));
      }
    }
    record(`g(θ) vs ∂P/∂θ — ${label}`, worst, 1e-6);
  }
});

test('c equals the Christoffel-symbol form built from derivatives of M', () => {
  // c_i = Σ_jk Γ_ijk θ̇_j θ̇_k,  Γ_ijk = ½(∂M_ij/∂θ_k + ∂M_ik/∂θ_j − ∂M_jk/∂θ_i)   (MR Eq. 8.10)
  const h = 1e-5;
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states) {
      const n = robot.N;
      const dM = []; // dM[k][i][j] = ∂M_ij/∂θ_k
      for (let k = 0; k < n; k++) {
        const tp = s.theta.slice(), tm = s.theta.slice();
        tp[k] += h; tm[k] -= h;
        const Mp = M(robot, tp), Mm = M(robot, tm);
        dM.push(Mp.map((row, i) => row.map((v, j) => (v - Mm[i][j]) / (2 * h))));
      }
      const c = coriolisTorques(robot, linkKinematics(robot, s.theta), s.dtheta);
      for (let i = 0; i < n; i++) {
        let ci = 0;
        for (let j = 0; j < n; j++) for (let k = 0; k < n; k++) {
          ci += 0.5 * (dM[k][i][j] + dM[j][i][k] - dM[i][j][k]) * s.dtheta[j] * s.dtheta[k];
        }
        worst = Math.max(worst, Math.abs(ci - c[i]));
      }
    }
    record(`c(θ,θ̇) vs Christoffel symbols of M — ${label}`, worst, 1e-5);
  }
});

test('single rigid body: G V̇ − ad_Vᵀ G V equals the sum over point masses', () => {
  // Seven point masses with their centre of mass at the frame origin
  // (the derivation of Topic 11 pp. 3–6 / MR §8.2.1, done numerically).
  const r = rng(11);
  let worst = 0;
  for (let trial = 0; trial < 20; trial++) {
    const m = r.vec(7, 0.2, 2), pts = Array.from({ length: 7 }, () => r.vec(3, -1, 1));
    const mt = m.reduce((a, b) => a + b, 0);
    const com = [0, 1, 2].map((k) => pts.reduce((a, p, i) => a + m[i] * p[k], 0) / mt);
    pts.forEach((p) => { for (let k = 0; k < 3; k++) p[k] -= com[k]; });
    const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    pts.forEach((p, i) => {
      const d = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) I[a][b] += m[i] * ((a === b ? d : 0) - p[a] * p[b]);
    });
    const V = r.vec(6, -2, 2), Vd = r.vec(6, -2, 2);
    const w = V.slice(0, 3), v = V.slice(3), wd = Vd.slice(0, 3), vd = Vd.slice(3);
    const tau = [0, 0, 0], f = [0, 0, 0];
    pts.forEach((p, i) => {
      // r̈ = v̇ + ω̇×r + ω×v + ω×(ω×r)
      const a1 = cross(wd, p), a2 = cross(w, v), a3 = cross(w, cross(w, p));
      const fi = [0, 1, 2].map((k) => m[i] * (vd[k] + a1[k] + a2[k] + a3[k]));
      const ti = cross(p, fi);
      for (let k = 0; k < 3; k++) { f[k] += fi[k]; tau[k] += ti[k]; }
    });
    const F = referenceEquations.wrench({
      G: spatialInertia(I, mt), V, Vd, AdTnext: null, Fnext: zerosVec(6), hasFext: false,
      // AdTnext is multiplied by a zero wrench, so give it an identity-sized zero matrix
      ...{ AdTnext: Array.from({ length: 6 }, () => zerosVec(6)) },
    });
    worst = Math.max(worst, maxDiff(F, [...tau, ...f]));
  }
  record('single-body Newton–Euler vs point-mass sum', worst, 1e-11);
});

test('forward dynamics inverts inverse dynamics (with wrenches on every link)', () => {
  const r = rng(3);
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states.slice(0, 40)) {
      const tau = r.vec(robot.N, -10, 10);
      const Fext = Array.from({ length: robot.N + 1 }, () => r.vec(6, -5, 5));
      const { ddtheta } = forwardDynamics(robot, s.theta, s.dtheta, tau, { Fext });
      const back = inverseDynamics(robot, s.theta, s.dtheta, ddtheta, { gravity: robot.gravity, Fext });
      worst = Math.max(worst, maxDiff(back, tau));
    }
    record(`forward ∘ inverse dynamics round trip — ${label}`, worst, 1e-9, { unit: 'N·m' });
  }
});

test('τ = M θ̈ + c + g + Σ D_i J_sᵀ F^ext reproduces full RNE', () => {
  const r = rng(5);
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states.slice(0, 40)) {
      const dd = r.vec(robot.N, -5, 5);
      const Fext = Array.from({ length: robot.N + 1 }, () => r.vec(6, -5, 5));
      const kin = linkKinematics(robot, s.theta);
      const full = rne(robot, kin, s.dtheta, dd, { gravity: robot.gravity, Fext }).tau;
      const Mq = massMatrix(robot, kin), c = coriolisTorques(robot, kin, s.dtheta), g = gravityTorques(robot, kin);
      const ext = externalTorques(robot, s.theta, Fext);
      const sum = matVec(Mq, dd).map((v, i) => v + c[i] + g[i] + ext[i]);
      worst = Math.max(worst, maxDiff(full, sum));
    }
    record(`configuration-space equation vs RNE — ${label}`, worst, 1e-10, { unit: 'N·m' });
  }
});

test('with θ̇ = 0 inverse dynamics returns g(θ), and the home-pose hold torques are right', () => {
  let worst = 0;
  for (const { robot, states } of sets) {
    for (const s of states.slice(0, 30)) {
      const z = zerosVec(robot.N);
      const id = inverseDynamics(robot, s.theta, z, z, { gravity: robot.gravity });
      worst = Math.max(worst, maxDiff(id, gravityTorques(robot, linkKinematics(robot, s.theta))));
    }
  }
  record('inverse dynamics at rest equals g(θ)', worst, 1e-12);

  // Hand calculation for the default arm stretched out horizontally (home):
  // the waist axis is parallel to gravity (no torque); the shoulder carries
  // the weight of links 2 and 3 at their horizontal distances; the elbow
  // carries link 3 only. Positive θ lifts, so the hold torques are positive.
  const robot = defaultRobot();
  const g = gravityTorques(robot, linkKinematics(robot, [0, 0, 0]));
  const expected = [0, 9.81 * (2.3 * 0.19 + 1.3 * 0.53), 9.81 * 1.3 * 0.15];
  record('static hold at home vs hand calculation', maxDiff(g, expected), 1e-12, {
    unit: 'N·m', note: `g(home) = [${g.map((x) => x.toFixed(4)).join(', ')}] N·m`,
  });
});
