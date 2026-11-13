// Equation-editor acceptance checks.
import test from 'node:test';
import assert from 'node:assert/strict';
import { linkKinematics } from '../src/dynamics/rne.js';
import { coriolisTorques, gravityTorques, massMatrix } from '../src/dynamics/terms.js';
import { compile } from '../src/equations/compile.js';
import { EquationError, parse } from '../src/equations/parser.js';
import { EquationSet, makeRandom, SLOTS } from '../src/equations/registry.js';
import { runChecks } from '../src/sim/checks.js';
import { Simulation } from '../src/sim/simulation.js';
import { loadFixtures, maxDiff, record, useResultsFile } from './helpers.js';

useResultsFile(import.meta.url);
const scale = (a) => (typeof a === 'number' ? Math.abs(a) : a.reduce((m, x) => Math.max(m, scale(x)), 0));

test('every seeded expression reproduces its reference', () => {
  const set = new EquationSet(['model', 'plant', 'shared'], 3);
  const r = makeRandom(99);
  let worstAbs = 0, worstRel = 0;
  for (const slot of SLOTS) {
    const seeded = set.compileSeed(slot.id);
    for (let k = 0; k < 100; k++) {
      const sample = slot.sample(r, 3);
      const want = slot.reference(sample), d = maxDiff(seeded(sample), want);
      worstAbs = Math.max(worstAbs, d);
      worstRel = Math.max(worstRel, d / (1 + scale(want)));
    }
  }
  record(`seeded editor text vs reference, ${SLOTS.length} equations × 100 random inputs (relative)`, worstRel, 1e-12, {
    note: `largest absolute difference ${worstAbs.toExponential(2)} on values up to ~10³`,
  });
});

test('M, c, g computed entirely from the editor text match the built-in versions', () => {
  // Wrap each seed in parentheses: different text, same maths, so the
  // compiled-expression path is what runs.
  const set = new EquationSet(['model', 'plant'], 3);
  for (const id of ['rne.baseAccel', 'rne.twist', 'rne.accel', 'rne.wrench', 'rne.torque']) {
    assert.equal(set.set(id, `(${set.text(id)})`).state, 'match');
  }
  const eqs = set.rneEquations();
  let worst = 0;
  for (const { robot, states } of loadFixtures()) {
    for (const s of states.slice(0, 50)) {
      const kin = linkKinematics(robot, s.theta);
      worst = Math.max(worst,
        maxDiff(massMatrix(robot, kin, eqs), massMatrix(robot, kin)),
        maxDiff(coriolisTorques(robot, kin, s.dtheta, eqs), coriolisTorques(robot, kin, s.dtheta)),
        maxDiff(gravityTorques(robot, kin, eqs), gravityTorques(robot, kin)));
    }
  }
  record('M, c, g via compiled editor text vs built-in RNE', worst, 1e-12);
});

test('a known-wrong edit is flagged, and the checks show what broke', () => {
  const start = { controller: 'zero', initial: { theta: [0.3, 1.0, -0.5], dtheta: [1.5, 0, 0] } };
  const [slipP5, noTranspose] = SLOTS.find((s) => s.id === 'rne.wrench').wrong;

  // (a) The Topic 11 p. 5 form, diag([ω],[v]). It drops the force m ω×v.
  // That force is always perpendicular to v, so it does NO work: energy is
  // still conserved even though the motion is wrong. The Christoffel check is
  // what catches it.
  const a = new Simulation(start), clean = new Simulation(start);
  const statusA = a.equations.plant.set('rne.wrench', slipP5.text);
  assert.equal(statusA.state, 'differs');
  record('wrong edit diag([ω],[v]): distance from reference on random inputs', statusA.maxDiff, 1e-3, { dir: '>=' });
  assert.ok(a.step(2000), a.fault?.message);
  clean.step(2000);
  const checksA = Object.fromEntries(runChecks(a, 'plant').map((c) => [c.id, c]));
  assert.equal(checksA.coriolis.pass, false, 'Christoffel check should fail');
  assert.equal(checksA.gravity.pass, true, 'gravity does not involve the velocity term');
  assert.equal(checksA.symmetric.pass, true);
  record('diag([ω],[v]) edit: Christoffel check error (limit 1e-4 means FAIL is expected)', checksA.coriolis.value, 1e-3, { dir: '>=', unit: 'N·m' });
  record('diag([ω],[v]) edit: energy balance error after 2 s (still conserved)', Math.abs(a.snapshot().energyBalance), 1e-6, { unit: 'J' });
  record('diag([ω],[v]) edit: joint-angle difference from the true motion after 2 s', maxDiff(a.theta, clean.theta), 1e-2, { dir: '>=', unit: 'rad' });

  // (b) Missing transpose on ad: also wrong, and also workless, because
  // V·(ad_V X) and V·(ad_Vᵀ X) both vanish for the terms involved.
  const b = new Simulation(start);
  assert.equal(b.equations.plant.set('rne.wrench', noTranspose.text).state, 'differs');
  b.step(2000);
  const checksB = Object.fromEntries(runChecks(b, 'plant').map((c) => [c.id, c]));
  assert.equal(checksB.coriolis.pass, false);
  record('missing-transpose edit: energy balance error after 2 s (still conserved)', Math.abs(b.snapshot().energyBalance), 1e-6, { unit: 'J' });

  // (c) Deleting the Coriolis term from the ACCELERATION recursion changes
  // G·V̇, which does do work: now the energy balance fails too.
  const c = new Simulation(start);
  const noCoriolis = SLOTS.find((s) => s.id === 'rne.accel').wrong[0];
  assert.equal(c.equations.plant.set('rne.accel', noCoriolis.text).state, 'differs');
  c.step(2000);
  const checksC = Object.fromEntries(runChecks(c, 'plant').map((x) => [x.id, x]));
  assert.equal(checksC.coriolis.pass, false);
  assert.equal(checksC.energy.pass, false, 'energy balance should fail with the acceleration term deleted');
  record('deleted Coriolis term in V̇_i: energy balance error after 2 s', c.fault ? Infinity : Math.abs(c.snapshot().energyBalance), 1e-2, { dir: '>=', unit: 'J' });

  // the controller's model was not touched by either edit
  for (const c of runChecks(a, 'model')) assert.ok(c.pass, `model: ${c.label}`);
  // undo: everything passes again
  a.equations.plant.reset('rne.wrench');
  a.reset();
  a.step(2000);
  for (const c of runChecks(a, 'plant')) assert.ok(c.pass, `after reset: ${c.label} = ${c.value}`);
});

test('a syntax error or wrong-size result does not stop the simulation', () => {
  const sim = new Simulation(), clean = new Simulation();
  sim.step(100); clean.step(100);
  for (const bad of ['G*Vd - ad(V', 'G*Vd +* V', 'G*Vd - nonsense(V)', 'G', 'G*Vd - ad(V)\'*G*V + Fext(9)', '']) {
    const status = sim.equations.plant.set('rne.wrench', bad);
    assert.equal(status.state, 'error', `"${bad}" should be rejected`);
    assert.match(status.message, /still running the last working version/);
  }
  assert.ok(sim.step(400));
  clean.step(400);
  assert.equal(sim.fault, null);
  record('state difference after rejected edits vs an untouched run', maxDiff(sim.x, clean.x), 0);
});

test('the parser only accepts whitelisted names and syntax', () => {
  const vars = ['A', 'V'];
  const rejected = [
    'constructor', '__proto__', 'this', 'window', 'globalThis', 'A.constructor', 'A["x"]', '"str"', '`x`',
    'eval(1)', 'Function(1)', 'constructor(1)', 'toString(1)', 'A = 1', 'A; V', '(() => 1)()', 'new(1)',
    'hasOwnProperty(1)', '__proto__(1)', 'valueOf(A)', 'import(1)', 'A.^.V', '[', '1 2', 'zeros(1e9)', 'eye(-1)',
  ];
  for (const text of rejected) {
    assert.throws(() => compile(text, vars).evaluate({ A: [1, 2, 3, 4, 5, 6], V: [1, 2, 3, 4, 5, 6] }), EquationError, `"${text}" should be rejected`);
  }
  assert.throws(() => parse('('.repeat(100) + '1' + ')'.repeat(100)), /nested too deeply/);
  assert.throws(() => parse('1+'.repeat(1500) + '1'), /too long/);
  // and the things that should work, do
  const ok = (text, scope, want) => assert.ok(maxDiff(compile(text, Object.keys(scope)).evaluate(scope), want) < 1e-12, text);
  ok('-2^2', {}, -4);
  ok('2^3^2', {}, 512);
  ok("a'*b", { a: [1, 2, 3], b: [4, 5, 6] }, 32);
  ok('a.*b', { a: [1, 2, 3], b: [4, 5, 6] }, [4, 10, 18]);
  ok('V(4:6)', { V: [1, 2, 3, 4, 5, 6] }, [4, 5, 6]);
  ok('M(2,:)', { M: [[1, 2], [3, 4]] }, [[3, 4]]);
  ok('[1, 2; 3, 4]*[1; 1]', {}, [3, 7]);
  ok('[eye(2), zeros(2,1); 0, 0, 5]', {}, [[1, 0, 0], [0, 1, 0], [0, 0, 5]]);
  ok('skew(w)*w', { w: [1, 2, 3] }, [0, 0, 0]);
  ok('2*pi % a comment', {}, 2 * Math.PI);
});

test('an edit to the controller model leaves the true plant untouched', () => {
  const sim = new Simulation();
  const kin = linkKinematics(sim.robot, [0.2, 0.5, -0.4]);
  const before = massMatrix(sim.robot, kin, sim.plant.eqs);
  assert.equal(sim.equations.model.set('inertia', 'blkdiag(I, 2*m*eye(3))').state, 'differs');
  sim.touch();
  assert.equal(maxDiff(massMatrix(sim.robot, kin, sim.plant.eqs), before), 0);
  const hatKin = linkKinematics(sim.hatRobot, [0.2, 0.5, -0.4]);
  assert.ok(maxDiff(massMatrix(sim.hatRobot, hatKin, sim.hat.eqs), before) > 0.01);
});
