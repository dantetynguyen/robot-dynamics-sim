// benchmark.mjs — how much faster than real time does the simulation run?
// Usage: npm run bench
// Prints simulated seconds per wall-clock second for three workloads. The UI
// refreshes both predictions about 10 times per simulated second.
import { Simulation } from '../src/sim/simulation.js';

// One second of simulated time, as the UI spends it at 1x speed:
// 1000 RK4 steps, plus `refresh` re-computations of BOTH predicted paths
// (1.5 s horizon each).
function run(label, setup, { refresh }) {
  const sim = new Simulation({ controller: 'ct' });
  setup(sim);
  sim.touch();
  sim.step(200); // warm up the JIT
  const simSeconds = 3;
  const chunks = Math.max(1, refresh) * simSeconds, stepsPerChunk = Math.round(simSeconds / sim.config.dt / chunks);
  const t0 = performance.now();
  for (let k = 0; k < chunks; k++) {
    sim.step(stepsPerChunk);
    if (refresh) { sim.predict({ plant: 'true' }); sim.predict({ plant: 'belief' }); }
  }
  const wall = (performance.now() - t0) / 1000;
  if (sim.fault) throw new Error(sim.fault.message);
  const ratio = simSeconds / wall;
  console.log(`${label.padEnd(74)} ${ratio.toFixed(1)}x real time`);
  return ratio;
}

const wrap = (set, ids) => { for (const id of ids) set.set(id, `(${set.text(id)})`); };
const RNE = ['rne.baseAccel', 'rne.twist', 'rne.accel', 'rne.wrench', 'rne.torque', 'inertia'];
const editOne = (sim) => wrap(sim.equations.plant, ['rne.wrench']);
const editAll = (sim) => {
  wrap(sim.equations.plant, RNE);
  wrap(sim.equations.model, RNE);
  wrap(sim.equations.shared, ['ctrl.ct']);
};

export function benchmark() {
  const probe = new Simulation();
  console.log(`dt = ${probe.config.dt * 1000} ms RK4, computed torque; predictions: ${probe.config.prediction.horizon} s horizon at ${probe.config.prediction.dt * 1000} ms, both paths`);
  return {
    plain: run('reference equations, no prediction', () => {}, { refresh: 0 }),
    predict: run('reference equations + both predictions refreshed 5x per second', () => {}, { refresh: 5 }),
    oneEdit: run('one edited plant equation (F_i) + both predictions 5x per second', editOne, { refresh: 5 }),
    allEdits: run('all 13 RNE/inertia/controller equations edited + both predictions 5x/s', editAll, { refresh: 5 }),
    allEditsNoPred: run('all 13 equations edited, no prediction', editAll, { refresh: 0 }),
  };
}

if (typeof process !== 'undefined' && process.argv[1] && process.argv[1].endsWith('benchmark.mjs')) {
  console.log(`Node ${process.version}`);
  benchmark();
}
