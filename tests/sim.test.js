// Simulation behaviour acceptance checks.
import test from 'node:test';
import assert from 'node:assert/strict';
import { secondOrderResponse } from '../src/control/controllers.js';
import { runChecks } from '../src/sim/checks.js';
import { Simulation } from '../src/sim/simulation.js';
import { record, useResultsFile } from './helpers.js';

useResultsFile(import.meta.url);
const DEG = Math.PI / 180;

test('zero torque, zero damping: energy is conserved over 10 s at the default step', () => {
  const sim = new Simulation({ controller: 'zero', initial: { theta: [0.3, 60 * DEG, -30 * DEG], dtheta: [1.5, 0, 0] } });
  const E0 = sim.snapshot().energy.E;
  let worst = 0, maxSpeed = 0;
  for (let k = 0; k < 100; k++) {
    assert.ok(sim.step(100), sim.fault?.message);
    worst = Math.max(worst, Math.abs(sim.snapshot().energy.E - E0));
    maxSpeed = Math.max(maxSpeed, ...sim.dtheta.map(Math.abs));
  }
  // Drift as a fraction of the energy actually sloshing around: the swing of
  // kinetic energy during the run.
  const swing = Math.max(...sim.log.map((r) => r.K)) - Math.min(...sim.log.map((r) => r.K));
  record('energy drift over 10 s, dt = 1 ms (fraction of kinetic-energy swing)', worst / swing, 1e-3, {
    note: `|ΔE|max = ${worst.toExponential(2)} J, kinetic-energy swing = ${swing.toFixed(2)} J, E0 = ${E0.toFixed(3)} J, peak joint speed = ${maxSpeed.toFixed(1)} rad/s`,
  });
  record('energy drift over 10 s as a fraction of |E0|', worst / Math.abs(E0), 1e-3);
});

for (const zeta of [1, 0.5, 1.6]) {
  test(`computed torque, perfect model: error follows the second-order prediction (ζ = ${zeta})`, () => {
    const wn = 8;
    const sim = new Simulation({
      controller: 'ct', gains: { mode: 'bandwidth', wn, zeta, Ki: [0, 0, 0] },
      // start away from the reference, and moving, so all three joints have a transient
      initial: { theta: [0.25, 20 * DEG + 0.2, -40 * DEG - 0.3], dtheta: [0.5, -0.4, 0.3] },
    });
    const ref0 = sim.reference(0);
    const e0 = ref0.theta.map((r, i) => r - sim.theta[i]);
    const de0 = ref0.dtheta.map((r, i) => r - sim.dtheta[i]);
    let worst = 0;
    for (let k = 0; k < 500; k++) {
      assert.ok(sim.step(10), sim.fault?.message);
      const ref = sim.reference(sim.t);
      for (let i = 0; i < 3; i++) {
        const predicted = secondOrderResponse(wn, zeta, e0[i], de0[i], sim.t);
        worst = Math.max(worst, Math.abs(ref.theta[i] - sim.theta[i] - predicted));
      }
    }
    record(`computed-torque error vs second-order prediction over 5 s (ωn = ${wn}, ζ = ${zeta})`, worst, 1e-6, {
      unit: 'rad', note: `initial errors [${e0.map((x) => x.toFixed(2)).join(', ')}] rad while tracking the default quintic move`,
    });
  });
}

test('dropping ĝ from computed torque leaves a steady-state droop; PD + gravity does not', () => {
  const hold = { trajectory: { type: 'hold', start: [0, 30 * DEG, -30 * DEG] }, initial: { theta: [0, 30 * DEG, -30 * DEG], dtheta: [0, 0, 0] } };
  const ok = new Simulation({ ...hold, controller: 'pdg' });
  const bad = new Simulation({ ...hold, controller: 'ct', hat: { dropG: true } });
  ok.step(4000); bad.step(4000);
  const errOk = Math.max(...ok.snapshot().e.map(Math.abs)), errBad = Math.max(...bad.snapshot().e.map(Math.abs));
  record('static hold error with PD + gravity compensation', errOk, 1e-9, { unit: 'rad' });
  record('steady-state droop when ĝ is dropped (must be visible)', errBad, 1e-2, { dir: '>=', unit: 'rad' });
});

test('a diverging simulation stops with a fault instead of throwing', () => {
  // negative derivative gain = energy pumped in every cycle
  const sim = new Simulation({ controller: 'pd', gains: { mode: 'manual', Kp: [400, 400, 400], Kd: [-60, -60, -60], Ki: [0, 0, 0] } });
  let ok = true;
  for (let k = 0; k < 200 && ok; k++) ok = sim.step(100);
  assert.equal(ok, false);
  assert.match(sim.fault.message, /diverged/);
  // fixing the gain and pressing reset brings it back
  sim.config.gains.Kd = [40, 40, 40];
  sim.reset();
  assert.equal(sim.fault, null);
  assert.ok(sim.step(500));
});

test('live checks all pass for the reference equations', () => {
  const sim = new Simulation();
  sim.step(700);
  for (const which of ['plant', 'model']) {
    for (const c of runChecks(sim, which)) assert.ok(c.pass, `${which}: ${c.label} = ${c.value}`);
  }
});

test('save and load restores parameters and edited equations', () => {
  const a = new Simulation({ controller: 'pd', hat: { massScale: 0.8 } });
  a.equations.plant.set('friction', 'b .* dtheta + 0.3*tanh(dtheta/0.02)');
  a.config.wrenches.push({ enabled: true, link: 3, frame: 'aligned', W: [0, 0, 0, 0, 0, -10] });
  const saved = JSON.parse(JSON.stringify(a.toJSON()));
  const b = new Simulation();
  assert.deepEqual(b.load(saved), []);
  assert.equal(b.config.controller, 'pd');
  assert.equal(b.config.hat.massScale, 0.8);
  assert.equal(b.equations.plant.text('friction'), 'b .* dtheta + 0.3*tanh(dtheta/0.02)');
  a.reset(); a.step(300); b.step(300);
  assert.deepEqual(a.theta, b.theta);
  assert.deepEqual(b.load({ nonsense: true }), ['not a robot-dynamics-sim setup file']);
});

test('prediction on the true plant matches what then happens; belief differs under model error', () => {
  const sim = new Simulation({ controller: 'ct', hat: { massScale: 0.6 }, prediction: { horizon: 1.0, dt: 0.001 } });
  sim.step(300);
  const truth = sim.predict({ plant: 'true' }), belief = sim.predict({ plant: 'belief' });
  sim.step(1000);
  const p = sim.snapshot().p, last = truth.points.at(-1), lastB = belief.points.at(-1);
  const d = Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]);
  const dB = Math.hypot(p[0] - lastB[0], p[1] - lastB[1], p[2] - lastB[2]);
  record('predicted (true plant) vs actual tool position 1 s later', d, 1e-9, { unit: 'm' });
  record('belief vs actual tool position with 40% mass error (must be visible)', dB, 1e-3, { dir: '>=', unit: 'm' });
});
