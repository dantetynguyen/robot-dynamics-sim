// checks.js — live physical consistency checks.
//
// Each check compares what the (possibly edited) equations produce against a
// property that must hold for any correct rigid-body model. When an equation
// is edited wrongly, the pattern of which checks fail points at what the
// equation was responsible for.
//
// runChecks(sim, 'plant' | 'model') returns a list of
//   { id, label, value, limit, pass, why }
// `why` is a one-line explanation shown in the UI.

import { jacobianSpace } from '../dynamics/kinematics.js';
import { linkKinematics, rne } from '../dynamics/rne.js';
import { coriolisTorques, gravityTorques, massMatrix, massMatrixKE, potentialEnergy } from '../dynamics/terms.js';
import { asymmetry, eigSym, matVec, maxAbs, zerosVec } from '../math/linalg.js';
import { adjoint, transInv } from '../math/lie.js';
import { buildRobot } from '../robot/model.js';
import { power, staticTorques } from '../wrench/wrench.js';

function diffMax(a, b) {
  let m = 0;
  const walk = (x, y) => { if (Array.isArray(x)) x.forEach((xi, i) => walk(xi, y[i])); else m = Math.max(m, Math.abs(x - y)); };
  walk(a, b);
  return m;
}

export function runChecks(sim, which = 'plant') {
  const model = which === 'plant' ? sim.plant : sim.hat;
  const robot = model.robot, eqs = model.eqs, N = robot.N;
  // A clean copy of the same robot built with the reference inertia formula:
  // the yardstick the edited equations are measured against.
  const ref = buildRobot(which === 'plant' ? sim.config.spec : sim.hatSpec);
  const theta = sim.theta;
  // Use the current joint rates, or a fixed test velocity when nearly at rest
  // so the velocity-dependent checks still say something.
  const dtheta = maxAbs(sim.dtheta) > 0.05 ? sim.dtheta : [1.0, -0.7, 0.5].slice(0, N);
  const out = [];
  const add = (id, label, value, limit, why, dir = '<=') => {
    const pass = Number.isFinite(value) && (dir === '<=' ? value <= limit : value >= limit);
    out.push({ id, label, value, limit, pass, why, dir });
  };

  try {
    const kin = linkKinematics(robot, theta);
    const Mq = massMatrix(robot, kin, eqs);
    const scaleM = Math.max(1e-9, ...Mq.flat().map(Math.abs));

    add('symmetric', 'M(θ) is symmetric', asymmetry(Mq) / scaleM, 1e-9,
      'Kinetic energy ½θ̇ᵀMθ̇ is a quadratic form, so M must equal its transpose.');
    add('posdef', 'M(θ) is positive definite', eigSym(Mq.map((r, i) => r.map((v, j) => (v + Mq[j][i]) / 2))).values[N - 1], 1e-9,
      'Any motion has positive kinetic energy, so every eigenvalue of M is positive.', '>=');
    add('ke', 'M from RNE = M from kinetic energy', diffMax(Mq, massMatrixKE(ref, theta)) / scaleM, 1e-8,
      'N calls of RNE with unit accelerations must rebuild Σ J_iᵀ G_i J_i (Topic 13 p. 5).');

    // g(θ) against a numerical gradient of the potential energy
    const g = gravityTorques(robot, kin, eqs);
    const h = 1e-6;
    let gErr = 0;
    for (let j = 0; j < N; j++) {
      const tp = theta.slice(), tm = theta.slice();
      tp[j] += h; tm[j] -= h;
      gErr = Math.max(gErr, Math.abs((potentialEnergy(ref, tp) - potentialEnergy(ref, tm)) / (2 * h) - g[j]));
    }
    add('gravity', 'g(θ) = ∂P/∂θ', gErr, 1e-5,
      'Gravity torques are the gradient of potential energy.');

    // c(θ, θ̇) against Christoffel symbols of the reference mass matrix
    const hc = 1e-5;
    const dM = [];
    for (let k = 0; k < N; k++) {
      const tp = theta.slice(), tm = theta.slice();
      tp[k] += hc; tm[k] -= hc;
      const Mp = massMatrixKE(ref, tp), Mm = massMatrixKE(ref, tm);
      dM.push(Mp.map((row, i) => row.map((v, j) => (v - Mm[i][j]) / (2 * hc))));
    }
    const c = coriolisTorques(robot, kin, dtheta, eqs);
    let cErr = 0;
    for (let i = 0; i < N; i++) {
      let ci = 0;
      for (let j = 0; j < N; j++) for (let k = 0; k < N; k++) ci += 0.5 * (dM[k][i][j] + dM[j][i][k] - dM[i][j][k]) * dtheta[j] * dtheta[k];
      cErr = Math.max(cErr, Math.abs(ci - c[i]));
    }
    add('coriolis', 'c(θ,θ̇) = Christoffel symbols of M', cErr, 1e-4,
      'Velocity-product torques are fixed by how M changes with θ (MR Eq. 8.10).');

    if (which === 'plant') {
      const snap = sim.snapshot();
      add('energy', 'Energy balance E − E₀ − W', Math.abs(snap.energyBalance), 1e-4 * (1 + Math.abs(sim.E0)),
        'Energy only changes by the work done on the arm. Holds under every controller if the plant equations are right.');
    }

    // Wrench tools (shared equations): a fixed test wrench/twist pair.
    const transform = sim.equations.shared.fn('wrench.transform');
    const F0 = [0.7, -0.4, 0.9, 12, -7, 5], V0 = [0.3, 0.8, -0.5, 0.2, -0.6, 0.4];
    const Ttool = kin.T0[N];
    const Fb = transform({ Tab: Ttool, Fa: F0 });           // F_tool = Ad_{T_0,tool}ᵀ F_0
    const Vb = matVec(adjoint(transInv(Ttool)), V0);          // V_tool = Ad_{T_tool,0} V_0
    add('power', 'Power VᵀF same in {0} and tool frame', Math.abs(power(V0, F0) - power(Vb, Fb)), 1e-9,
      'Power is a physical quantity; it cannot depend on the frame used to write V and F (Topic 10 p. 7).');

    const Js = jacobianSpace(ref, theta);
    const zero = zerosVec(N);
    const Fext = new Array(N + 1).fill(null);
    Fext[N] = F0;
    const viaRne = rne(ref, linkKinematics(ref, theta), zero, zero, { gravity: null, Fext }).tau;
    add('statics', 'Statics τ = J_sᵀF agrees with RNE', diffMax(sim.equations.shared.fn('statics')({ Js, F: F0 }), viaRne), 1e-9,
      'Holding a tool wrench at rest must cost the same torque either way.');
    add('staticsRef', 'J_sᵀF (reference) agrees with this model\'s RNE',
      diffMax(staticTorques(Js, F0), rne(robot, kin, zero, zero, { gravity: null, Fext, eqs }).tau), 1e-8,
      'The backward pass must carry a tool wrench to the joints exactly as the Jacobian transpose does.');
  } catch (e) {
    out.push({ id: 'crash', label: 'Model can be evaluated', value: NaN, limit: 0, pass: false, why: e.message, dir: '<=' });
  }
  return out;
}
