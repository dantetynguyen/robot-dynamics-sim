// Kinematics acceptance checks: FK and J_s against the independent Python
// reference, J_s against finite differences of FK, and the two body-Jacobian
// routes against each other.
import test from 'node:test';
import { fkSpace, jacobianBody, jacobianBodyDirect, jacobianSpace, linkPoses } from '../src/dynamics/kinematics.js';
import { linkKinematics } from '../src/dynamics/rne.js';
import { matMul } from '../src/math/linalg.js';
import { screwAxis, screwGeometry, transInv, unskew } from '../src/math/lie.js';
import { loadFixtures, maxDiff, record, rng, useResultsFile } from './helpers.js';

useResultsFile(import.meta.url);
const sets = loadFixtures();

test('FK matches the Python reference at 100 random states', () => {
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states) worst = Math.max(worst, maxDiff(fkSpace(robot, s.theta), s.T));
    record(`FK vs Python reference — ${label}`, worst, 1e-9);
  }
});

test('space Jacobian matches the Python reference at 100 random states', () => {
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states) worst = Math.max(worst, maxDiff(jacobianSpace(robot, s.theta), s.Js));
    record(`J_s vs Python reference — ${label}`, worst, 1e-9);
  }
});

test('space Jacobian matches finite differences of FK', () => {
  // [V_s] = Ṫ T⁻¹, so column j of J_s is unbracket( ∂T/∂θ_j · T⁻¹ ).
  const h = 1e-6;
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states) {
      const Js = jacobianSpace(robot, s.theta);
      const Tinv = transInv(fkSpace(robot, s.theta));
      for (let j = 0; j < robot.N; j++) {
        const tp = s.theta.slice(), tm = s.theta.slice();
        tp[j] += h; tm[j] -= h;
        const Tp = fkSpace(robot, tp), Tm = fkSpace(robot, tm);
        const dT = Tp.map((row, r) => row.map((v, c) => (v - Tm[r][c]) / (2 * h)));
        const Vb = matMul(dT, Tinv);
        const col = [...unskew(Vb), Vb[0][3], Vb[1][3], Vb[2][3]];
        worst = Math.max(worst, maxDiff(col, Js.map((row) => row[j])));
      }
    }
    record(`J_s vs finite-difference FK — ${label}`, worst, 1e-6);
  }
});

test('body Jacobian: Ad_{T⁻¹} J_s equals the direct body-screw formula', () => {
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states) worst = Math.max(worst, maxDiff(jacobianBody(robot, s.theta), jacobianBodyDirect(robot, s.theta)));
    record(`J_b two routes — ${label}`, worst, 1e-10);
  }
});

test('link frames from the RNE one-joint transforms equal the PoE link poses', () => {
  for (const { label, robot, states } of sets) {
    let worst = 0;
    for (const s of states.slice(0, 20)) worst = Math.max(worst, maxDiff(linkKinematics(robot, s.theta).T0, linkPoses(robot, s.theta)));
    record(`T_0i chain vs PoE — ${label}`, worst, 1e-10);
  }
});

test('screw axis is unchanged when q slides along the axis', () => {
  const r = rng(7);
  let worst = 0;
  for (let k = 0; k < 50; k++) {
    const s = r.vec(3, -1, 1), q = r.vec(3, -1, 1), lam = r.uniform(-2, 2);
    const n = Math.hypot(...s), sh = s.map((x) => x / n);
    const S1 = screwAxis(sh, q), S2 = screwAxis(sh, q.map((x, i) => x + lam * sh[i]));
    worst = Math.max(worst, maxDiff(S1, S2));
    const g = screwGeometry(S1);
    worst = Math.max(worst, maxDiff(screwAxis(g.sHat, g.q, g.h), S1));
  }
  record('screw axis invariance + geometry round trip', worst, 1e-12);
});
