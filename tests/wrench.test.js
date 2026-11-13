// Wrench acceptance checks.
import test from 'node:test';
import { jacobianSpace, linkPoses, pointJacobian } from '../src/dynamics/kinematics.js';
import { linkKinematics, rne } from '../src/dynamics/rne.js';
import { externalTorques, forwardDynamics, gravityTorques } from '../src/dynamics/terms.js';
import { dot, matTVec, matVec, norm, solve, zerosVec } from '../src/math/linalg.js';
import { adjoint, matrixExp6, rpToTrans, transInv, matrixExp3 } from '../src/math/lie.js';
import {
  appliedToNotes, pointEllipsoids, power, reciprocalBasis, screwToWrench, splitJointWrench,
  staticTorques, wrenchToScrew, wrenchToSpace, wrenchTransform,
} from '../src/wrench/wrench.js';
import { loadFixtures, maxDiff, record, rng, useResultsFile } from './helpers.js';

useResultsFile(import.meta.url);
const sets = loadFixtures();

function randomTransform(r) {
  const axis = r.vec(3, -1, 1), n = norm(axis);
  return rpToTrans(matrixExp3(axis.map((x) => x / n), r.uniform(-3, 3)), r.vec(3, -1, 1));
}

test('power VᵀF is the same in every frame', () => {
  const r = rng(21);
  let worst = 0;
  for (let k = 0; k < 200; k++) {
    const Tab = randomTransform(r);
    const Vb = r.vec(6, -1, 1), Fa = r.vec(6, -1, 1);
    const Va = matVec(adjoint(Tab), Vb);     // V_a = Ad_{T_ab} V_b
    const Fb = wrenchTransform(Tab, Fa);     // F_b = Ad_{T_ab}ᵀ F_a
    worst = Math.max(worst, Math.abs(power(Va, Fa) - power(Vb, Fb)));
    // and through a third frame
    const Tbc = randomTransform(r);
    const Vc = matVec(adjoint(transInv(Tbc)), Vb), Fc = wrenchTransform(Tbc, Fb);
    worst = Math.max(worst, Math.abs(power(Vc, Fc) - power(Va, Fa)));
  }
  record('power invariance across three frames', worst, 1e-12, { unit: 'W' });
});

test('J_sᵀ F equals RNE run with that wrench as F^ext', () => {
  const r = rng(22);
  for (const { label, robot, states } of sets) {
    let wTip = 0, wLink = 0;
    for (const s of states.slice(0, 50)) {
      const kin = linkKinematics(robot, s.theta), z = zerosVec(robot.N);
      const Js = jacobianSpace(robot, s.theta);
      const F = r.vec(6, -10, 10);
      const Fext = new Array(robot.N + 1).fill(null);
      Fext[robot.N] = F;
      wTip = Math.max(wTip, maxDiff(rne(robot, kin, z, z, { gravity: null, Fext }).tau, staticTorques(Js, F)));
      // a wrench on a middle link reaches only the joints before it (D_i)
      const i = Math.floor(r.uniform(0, robot.N));
      const FextL = new Array(robot.N + 1).fill(null);
      FextL[i] = F;
      const tauL = rne(robot, kin, z, z, { gravity: null, Fext: FextL }).tau;
      const expect = staticTorques(Js, F).map((v, j) => (j <= i ? v : 0));
      wLink = Math.max(wLink, maxDiff(tauL, expect), maxDiff(tauL, externalTorques(robot, s.theta, FextL)));
    }
    record(`statics τ = J_sᵀF vs RNE (tip wrench) — ${label}`, wTip, 1e-10, { unit: 'N·m' });
    record(`per-link wrench D_i J_sᵀF vs RNE — ${label}`, wLink, 1e-10, { unit: 'N·m' });
  }
});

test('wrenches in null(J_sᵀ) produce zero joint torque and no motion', () => {
  for (const { label, robot, states } of sets) {
    let wTau = 0, wAcc = 0, dims = new Set();
    for (const s of states.slice(0, 50)) {
      const Js = jacobianSpace(robot, s.theta);
      const { basis } = reciprocalBasis(Js);
      dims.add(basis.length);
      const kin = linkKinematics(robot, s.theta), z = zerosVec(robot.N);
      for (const b of basis) {
        const F = b.map((x) => 25 * x);
        const Fext = new Array(robot.N + 1).fill(null);
        Fext[robot.N] = F;
        wTau = Math.max(wTau, maxDiff(rne(robot, kin, z, z, { gravity: null, Fext }).tau, z));
        // released from rest with gravity off, the arm must not accelerate
        wAcc = Math.max(wAcc, maxDiff(forwardDynamics(robot, s.theta, z, z, { gravity: null, Fext }).ddtheta, z));
      }
    }
    record(`null(J_sᵀ) wrench (25 N scale) → joint torque — ${label}`, wTau, 1e-10, {
      unit: 'N·m', note: `null-space dimension(s) seen: ${[...dims].join(', ')}`,
    });
    record(`null(J_sᵀ) wrench → acceleration from rest — ${label}`, wAcc, 1e-8, { unit: 'rad/s²' });
  }
});

test('screw (axis, pitch, magnitude) form rebuilds the 6-vector', () => {
  const r = rng(23);
  let worst = 0;
  for (let k = 0; k < 300; k++) {
    let F = r.vec(6, -5, 5);
    if (k % 10 === 0) F = [F[0], F[1], F[2], 0, 0, 0];          // pure couple
    if (k % 10 === 1) F = [0, 0, 0, F[3], F[4], F[5]];          // pure force through the origin
    worst = Math.max(worst, maxDiff(screwToWrench(wrenchToScrew(F)), F));
  }
  record('wrench → screw → wrench round trip', worst, 1e-12);
});

test('static hold with an applied wrench gives θ̈ = 0', () => {
  const r = rng(24);
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states.slice(0, 50)) {
      const z = zerosVec(robot.N);
      // a wrench applied TO the tool, typed in the tool frame, moved to {0}
      const Wtool = r.vec(6, -8, 8);
      const Ttool = linkPoses(robot, s.theta)[robot.N];
      const Fext = new Array(robot.N + 1).fill(null);
      Fext[robot.N] = appliedToNotes(wrenchToSpace(Ttool, Wtool));
      const hold = gravityTorques(robot, linkKinematics(robot, s.theta)).map((g, j) => g + externalTorques(robot, s.theta, Fext)[j]);
      worst = Math.max(worst, maxDiff(forwardDynamics(robot, s.theta, z, hold, { Fext }).ddtheta, z));
    }
    record(`static hold θ̈ with applied tool wrench — ${label}`, worst, 1e-9, { unit: 'rad/s²' });
  }
});

test('force and manipulability ellipsoids are dual', () => {
  for (const { label, robot, states } of sets) {
    let wProd = 0, wVel = 0, wForce = 0;
    for (const s of states.slice(0, 50)) {
      const Js = jacobianSpace(robot, s.theta);
      const p = linkPoses(robot, s.theta)[robot.N].map((row) => row[3]).slice(0, 3);
      const Jp = pointJacobian(Js, p);
      const e = pointEllipsoids(Jp);
      for (let k = 0; k < 3; k++) {
        // same axis, reciprocal lengths
        wProd = Math.max(wProd, Math.abs(e.manipulability[k] * e.force[k] - 1));
        // tip of the velocity semi-axis is reached by a unit joint-rate vector
        const dth = solve(Jp, e.axes[k].map((x) => x * e.manipulability[k]));
        wVel = Math.max(wVel, Math.abs(norm(dth) - 1));
        // tip of the force semi-axis costs a unit joint-torque vector
        const tau = matTVec(Jp, e.axes[k].map((x) => x * e.force[k]));
        wForce = Math.max(wForce, Math.abs(norm(tau) - 1));
      }
    }
    record(`ellipsoid duality: (velocity axis)·(force axis) − 1 — ${label}`, wProd, 1e-9);
    record(`velocity semi-axis ↔ ‖θ̇‖ = 1 — ${label}`, wVel, 1e-6);
    record(`force semi-axis ↔ ‖τ‖ = 1 — ${label}`, wForce, 1e-6);
  }
});

test('joint wrench = motor part + bearing part, and F_iᵀ A_i = τ_i', () => {
  const r = rng(25);
  for (const { label, robot, states } of sets) {
    let wSum = 0, wTau = 0, wRecip = 0, wAxial = 0;
    for (const s of states.slice(0, 50)) {
      const kin = linkKinematics(robot, s.theta);
      const out = rne(robot, kin, s.dtheta, r.vec(robot.N, -5, 5), { gravity: robot.gravity });
      for (let i = 0; i < robot.N; i++) {
        const sp = splitJointWrench(out.F[i], robot.A[i]);
        wSum = Math.max(wSum, maxDiff(sp.motor.map((x, k) => x + sp.bearing[k]), out.F[i]));
        wTau = Math.max(wTau, Math.abs(dot(out.F[i], robot.A[i]) - out.tau[i]), Math.abs(sp.tauMotor - out.tau[i]));
        wRecip = Math.max(wRecip, Math.abs(dot(sp.bearing, robot.A[i])));   // bearing does no work on the joint twist
        wAxial = Math.max(wAxial, Math.abs(sp.residualAxialMoment));
      }
    }
    record(`motor + bearing = F_i — ${label}`, wSum, 1e-12);
    record(`F_iᵀ A_i = τ_i — ${label}`, wTau, 1e-12, { unit: 'N·m' });
    record(`bearing wrench reciprocal to joint twist — ${label}`, wRecip, 1e-11, { unit: 'W per rad/s' });
    record(`bearing moment about the joint axis — ${label}`, wAxial, 1e-11, { unit: 'N·m' });
  }
});
