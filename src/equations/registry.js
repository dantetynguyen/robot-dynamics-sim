// registry.js — the list of equations that can be edited from the UI, and the
// EquationSet class that holds the current (possibly edited) version of each.
//
// TO ADD AN EDITABLE EQUATION:
//   1. Add an entry to SLOTS below: an id, the seed text (the reference
//      equation in editor syntax), the variables it may use, the shape of its
//      result, a `reference` function (the same maths in plain JavaScript), and
//      a `sample` function that makes random inputs for validation.
//   2. Call `equationSet.fn('your.id')(scope)` where the equation is needed.
// The editor UI, validation ("does my edit match the reference?"), save/load
// and the tests all pick the new slot up automatically.
//
// Each slot has a `scope`:
//   'model'  — part of a dynamics model. There are two copies: one in the true
//              plant and one in the controller's model, so an edit can be made
//              to either or both.
//   'plant'  — exists only in the true plant (friction).
//   'shared' — one copy (controller laws, trajectories, wrench tools).

import { matVec, norm, zerosVec } from '../math/linalg.js';
import { adjoint, matrixExp3, rpToTrans } from '../math/lie.js';
import { referenceEquations } from '../dynamics/rne.js';
import { spatialInertia } from '../robot/inertia.js';
import { staticTorques, wrenchTransform } from '../wrench/wrench.js';
import { compile, describe } from './compile.js';
import { EquationError } from './parser.js';

// ---------- random inputs for validation ----------

function makeRandom(seed) {
  let a = seed >>> 0;
  const next = () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const r = (lo = -1, hi = 1) => lo + (hi - lo) * next();
  r.vec = (n, lo = -1, hi = 1) => Array.from({ length: n }, () => r(lo, hi));
  r.transform = () => {
    const axis = r.vec(3), n = norm(axis) || 1;
    return rpToTrans(matrixExp3(axis.map((x) => x / n), r(-3, 3)), r.vec(3));
  };
  r.spd = (n) => { // random symmetric positive-definite matrix
    const B = Array.from({ length: n }, () => r.vec(n));
    return B.map((row, i) => row.map((_, j) => B.reduce((s, _b, k) => s + B[i][k] * B[j][k], 0) + (i === j ? 0.5 : 0)));
  };
  return r;
}

const transformSample = (r) => { const T = r.transform(); return { T, AdT: adjoint(T) }; };

// ---------- the slots ----------

export const SLOTS = [
  {
    id: 'rne.baseAccel', scope: 'model', group: 'Recursive Newton–Euler',
    title: 'Gravity as a base acceleration',
    lhs: 'V̇₀ =', seed: '[zeros(3,1); -g]',
    vars: { g: 'gravity vector in {0}, 3-vector (default [0; 0; -9.81])' },
    out: { kind: 'vec', n: 6 },
    cite: 'Topic 11 p. 14; MR §8.3.2',
    note: 'The base is treated as accelerating opposite to gravity; the forward pass then carries gravity to every link for free.',
    reference: referenceEquations.baseAccel,
    sample: (r) => ({ g: r.vec(3, -10, 10) }),
  },
  {
    id: 'rne.twist', scope: 'model', group: 'Recursive Newton–Euler',
    title: 'Forward pass: link twist',
    lhs: 'V_i =', seed: 'A*dtheta + Ad(T)*Vprev',
    vars: {
      A: 'screw axis of joint i in link frame {i}, 6-vector',
      dtheta: 'joint rate θ̇_i, number',
      T: 'T_{i,i-1}(θ_i), 4×4',
      Vprev: 'twist of the previous link V_{i-1}, 6-vector',
    },
    out: { kind: 'vec', n: 6 },
    cite: 'Topic 11 p. 12; MR Eq. 8.51',
    note: 'This joint\'s own motion plus the previous link\'s twist re-expressed in frame {i}.',
    reference: referenceEquations.twist,
    sample: (r) => ({ A: r.vec(6), dtheta: r(-3, 3), ...transformSample(r), Vprev: r.vec(6, -2, 2) }),
  },
  {
    id: 'rne.accel', scope: 'model', group: 'Recursive Newton–Euler',
    title: 'Forward pass: link acceleration',
    lhs: 'V̇_i =', seed: 'A*ddtheta + ad(V)*A*dtheta + Ad(T)*Vdprev',
    vars: {
      A: 'screw axis of joint i in {i}, 6-vector',
      dtheta: 'θ̇_i, number', ddtheta: 'θ̈_i, number',
      V: 'twist of this link V_i, 6-vector',
      T: 'T_{i,i-1}(θ_i), 4×4',
      Vdprev: 'acceleration of the previous link V̇_{i-1}, 6-vector',
    },
    out: { kind: 'vec', n: 6 },
    cite: 'Topic 11 p. 13; MR Eq. 8.52',
    note: 'The middle term is the Coriolis/centripetal part. Delete it and the Christoffel check fails.',
    reference: referenceEquations.accel,
    sample: (r) => ({ A: r.vec(6), dtheta: r(-3, 3), ddtheta: r(-5, 5), V: r.vec(6, -2, 2), ...transformSample(r), Vdprev: r.vec(6, -5, 5) }),
    wrong: [
      { label: 'Coriolis/centripetal term deleted', text: 'A*ddtheta + Ad(T)*Vdprev' },
    ],
  },
  {
    id: 'rne.wrench', scope: 'model', group: 'Recursive Newton–Euler',
    title: 'Backward pass: joint wrench',
    lhs: 'F_i =', seed: "G*Vd - ad(V)'*G*V + Ad(Tnext)'*Fnext + Ad(T0i)'*Fext",
    vars: {
      G: 'spatial inertia of link i in {i}, 6×6',
      V: 'twist V_i, 6-vector', Vd: 'acceleration V̇_i, 6-vector',
      Tnext: 'T_{i+1,i}, 4×4', Fnext: 'wrench of the next joint F_{i+1}, 6-vector',
      T0i: 'T_{0,i}, pose of {i} in {0}, 4×4',
      Fext: 'wrench link i exerts on the world, in {0}, 6-vector',
    },
    out: { kind: 'vec', n: 6 },
    cite: 'Topic 11 p. 13; MR Eq. 8.53 (the F^ext term is from the notes only)',
    note: 'First two terms: Newton–Euler for this one body. Third: load passed on to the next link. Fourth: external wrench.',
    reference: referenceEquations.wrench,
    sample: (r) => {
      const Tnext = r.transform();
      return {
        G: spatialInertia(r.spd(3), r(0.5, 4)), V: r.vec(6, -2, 2), Vd: r.vec(6, -5, 5),
        Tnext, AdTnext: adjoint(Tnext), Fnext: r.vec(6, -5, 5),
        T0i: r.transform(), Fext: r.vec(6, -5, 5), hasFext: true,
      };
    },
    // Ready-made wrong versions for the "break an equation" experiment.
    wrong: [
      {
        label: 'diag([ω],[v]) velocity term (the form on Topic 11 p. 5)',
        text: "G*Vd + blkdiag(skew(V(1:3)), skew(V(4:6)))*G*V + Ad(Tnext)'*Fnext + Ad(T0i)'*Fext",
      },
      {
        label: 'missing transpose on ad',
        text: "G*Vd - ad(V)*G*V + Ad(Tnext)'*Fnext + Ad(T0i)'*Fext",
      },
    ],
  },
  {
    id: 'rne.torque', scope: 'model', group: 'Recursive Newton–Euler',
    title: 'Backward pass: joint torque',
    lhs: 'τ_i =', seed: "F'*A",
    vars: { F: 'joint wrench F_i, 6-vector', A: 'screw axis of joint i in {i}, 6-vector' },
    out: { kind: 'num' },
    cite: 'Topic 11 p. 13; MR Eq. 8.54',
    note: 'The motor only supplies the part of F_i along its own screw; the bearing carries the rest.',
    reference: referenceEquations.torque,
    sample: (r) => ({ F: r.vec(6, -5, 5), A: r.vec(6) }),
  },
  {
    id: 'inertia', scope: 'model', group: 'Rigid body',
    title: 'Spatial inertia at the centre of mass',
    lhs: 'G =', seed: 'blkdiag(I, m*eye(3))',
    vars: { I: 'rotational inertia about the CoM, 3×3', m: 'link mass, number' },
    out: { kind: 'mat', rows: 6, cols: 6 },
    cite: 'Topic 11 p. 5; MR Eq. 8.32',
    note: 'Rotational inertia on top, mass on the bottom: the same top/bottom order as twists and wrenches.',
    reference: (s) => spatialInertia(s.I, s.m),
    sample: (r) => ({ I: r.spd(3), m: r(0.5, 4) }),
  },
  {
    id: 'friction', scope: 'plant', group: 'Rigid body',
    title: 'Joint friction torque',
    lhs: 'τ_fric =', seed: 'b .* dtheta',
    vars: { b: 'viscous damping coefficients, N-vector', dtheta: 'joint rates θ̇, N-vector' },
    out: { kind: 'vec', n: 'N' },
    cite: 'MR §8.9.4 (friction); the notes fold damping into h (Topic 11 p. 2)',
    note: 'Try b .* dtheta + 0.4*tanh(dtheta/0.02) for Coulomb friction.',
    reference: (s) => s.b.map((bi, i) => bi * s.dtheta[i]),
    sample: (r, N) => ({ b: r.vec(N, 0, 2), dtheta: r.vec(N, -3, 3) }),
  },
  {
    id: 'wrench.transform', scope: 'shared', group: 'Wrenches',
    title: 'Wrench change of frame',
    lhs: 'F_b =', seed: "Ad(Tab)'*Fa",
    vars: { Tab: 'T_ab, pose of frame {b} seen from {a}, 4×4', Fa: 'the wrench expressed in {a}, 6-vector' },
    out: { kind: 'vec', n: 6 },
    cite: 'Topic 10 p. 7; MR Eq. 3.95',
    note: 'Follows from power being the same in every frame. Drop the transpose and the power check fails.',
    reference: (s) => wrenchTransform(s.Tab, s.Fa),
    sample: (r) => ({ Tab: r.transform(), Fa: r.vec(6, -5, 5) }),
  },
  {
    id: 'statics', scope: 'shared', group: 'Wrenches',
    title: 'Statics',
    lhs: 'τ =', seed: "Js'*F",
    vars: { Js: 'space Jacobian at the current θ, 6×N', F: 'wrench the tool exerts on the world, in {0}, 6-vector' },
    out: { kind: 'vec', n: 'N' },
    cite: 'Topic 10 p. 8; MR Eq. 5.26',
    note: 'Virtual work: joint power equals tool power for every small motion.',
    reference: (s) => staticTorques(s.Js, s.F),
    sample: (r, N) => ({ Js: Array.from({ length: 6 }, () => r.vec(N)), F: r.vec(6, -5, 5) }),
  },
  {
    id: 'traj.cubic', scope: 'shared', group: 'Trajectory',
    title: 'Cubic time scaling',
    lhs: 's(t) =', seed: '3*(t/T)^2 - 2*(t/T)^3',
    vars: { t: 'time since the move started, 0 ≤ t ≤ T', T: 'move duration, s' },
    out: { kind: 'num' },
    cite: 'MR §9.2.2.1, Eqs. 9.9–9.11 (third-order polynomial time scaling)',
    note: 'Starts and ends at rest, but the acceleration jumps at both ends.',
    reference: (s) => { const u = s.t / s.T; return 3 * u * u - 2 * u * u * u; },
    sample: (r) => { const T = r(0.5, 4); return { T, t: r(0, T) }; },
  },
  {
    id: 'traj.quintic', scope: 'shared', group: 'Trajectory',
    title: 'Quintic time scaling',
    lhs: 's(t) =', seed: '10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5',
    vars: { t: 'time since the move started, 0 ≤ t ≤ T', T: 'move duration, s' },
    out: { kind: 'num' },
    cite: 'MR §9.2.2.1 (fifth-order polynomial time scaling)',
    note: 'Zero velocity AND zero acceleration at both ends, so the torque demand starts smoothly.',
    reference: (s) => { const u = s.t / s.T; return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5; },
    sample: (r) => { const T = r(0.5, 4); return { T, t: r(0, T) }; },
  },
  ...controllerSlots(),
];

// Controller laws are equations too, so adding a controller is adding a slot
// (plus one line in src/control/controllers.js).
function controllerSlots() {
  const vars = {
    e: 'position error θ_d − θ, N-vector', de: 'velocity error θ̇_d − θ̇, N-vector',
    ie: 'integral of the error ∫e dt, N-vector',
    theta: 'θ', dtheta: 'θ̇', thetad: 'reference θ_d', dthetad: 'θ̇_d', ddthetad: 'θ̈_d',
    Kp: 'proportional gains, N×N diagonal', Kd: 'derivative gains, N×N diagonal', Ki: 'integral gains, N×N diagonal',
    Mhat: "controller's mass matrix M̂(θ), N×N", chat: "controller's Coriolis torques ĉ(θ,θ̇)",
    ghat: "controller's gravity torques ĝ(θ)", what: 'torques for the applied wrenches the controller knows about, Σ D_i J_sᵀ F̂^ext',
    t: 'time, s',
  };
  const sample = (r, N) => {
    const v = () => r.vec(N, -2, 2);
    const d = () => r.vec(N, 5, 80).map((x, i, a) => a.map((_, j) => (i === j ? x : 0)));
    return {
      e: v(), de: v(), ie: v(), theta: v(), dtheta: v(), thetad: v(), dthetad: v(), ddthetad: v(),
      Kp: d(), Kd: d(), Ki: d(), Mhat: r.spd(N), chat: v(), ghat: v(), what: v(), t: r(0, 10),
    };
  };
  const mk = (id, title, seed, cite, note, reference) => ({
    id, scope: 'shared', group: 'Controller', title, lhs: 'τ =', seed, vars, out: { kind: 'vec', n: 'N' }, cite, note, reference, sample,
  });
  const add = (...vs) => vs[0].map((_, i) => vs.reduce((s, v) => s + v[i], 0));
  return [
    mk('ctrl.zero', 'Zero torque', '0*e', '—', 'Motors off: the arm falls under gravity. With no damping, energy must stay constant.',
      (s) => zerosVec(s.e.length)),
    mk('ctrl.pd', 'PD', 'Kp*e + Kd*de', 'MR Eq. 11.23 with K_i = 0',
      'A spring and a damper on each joint. Gravity is not cancelled, so the arm settles below its target.',
      (s) => add(matVec(s.Kp, s.e), matVec(s.Kd, s.de))),
    mk('ctrl.pdg', 'PD + gravity compensation', 'Kp*e + Kd*de + ghat', 'MR Eq. 11.38 with K_i = 0',
      'The model supplies the holding torque; feedback only has to correct errors.',
      (s) => add(matVec(s.Kp, s.e), matVec(s.Kd, s.de), s.ghat)),
    mk('ctrl.ct', 'Computed torque', 'Mhat*(ddthetad + Kd*de + Kp*e + Ki*ie) + chat + ghat + what', 'MR Eq. 11.37',
      'Inverse dynamics on the controller\'s model. With a perfect model each joint error obeys ë + K_d ė + K_p e = 0.',
      (s) => add(matVec(s.Mhat, add(s.ddthetad, matVec(s.Kd, s.de), matVec(s.Kp, s.e), matVec(s.Ki, s.ie))), s.chat, s.ghat, s.what)),
  ];
}

export const SLOT_BY_ID = Object.fromEntries(SLOTS.map((s) => [s.id, s]));

// ---------- helpers ----------

function maxDiff(a, b) {
  if (typeof a === 'number') return typeof b === 'number' ? Math.abs(a - b) : Infinity;
  if (!Array.isArray(b) || a.length !== b.length) return Infinity;
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, maxDiff(a[i], b[i]));
  return m;
}

function scale(a) {
  if (typeof a === 'number') return Math.abs(a);
  return a.reduce((m, x) => Math.max(m, scale(x)), 0);
}

// Coerce a compiled result into the shape the slot promises, or throw.
function shapeChecker(slot, N) {
  const out = slot.out;
  const n = out.n === 'N' ? N : out.n;
  return (value) => {
    if (out.kind === 'num') {
      if (typeof value === 'number') return value;
    } else if (out.kind === 'vec') {
      if (Array.isArray(value) && !Array.isArray(value[0]) && value.length === n) return value;
      if (Array.isArray(value) && Array.isArray(value[0]) && value.length === 1 && value[0].length === n) return value[0]; // a 1×n row
      if (n === 1 && typeof value === 'number') return [value];
    } else if (out.kind === 'mat') {
      if (Array.isArray(value) && Array.isArray(value[0]) && value.length === out.rows && value[0].length === out.cols) return value;
    }
    const want = out.kind === 'num' ? 'a number' : out.kind === 'vec' ? `a ${n}-vector` : `a ${out.rows}×${out.cols} matrix`;
    throw new EquationError(`result should be ${want} but is ${describe(value)}`);
  };
}

export const MATCH_TOLERANCE = 1e-9; // relative; an edit within this of the reference "matches"

// ---------- EquationSet ----------

// Holds one copy of some slots. Status of each slot is one of
//   'reference' — untouched seed text; the fast built-in version runs
//   'match'     — edited, and numerically equal to the reference on random inputs
//   'differs'   — edited and valid, but gives different numbers (maxDiff says by how much)
//   'error'     — does not parse / wrong size; the last good version keeps running
export class EquationSet {
  constructor(scopes, N) {
    this.N = N;
    this.slots = SLOTS.filter((s) => scopes.includes(s.scope));
    this.entries = new Map();
    for (const slot of this.slots) {
      // `uses`: which variables the current text reads, so callers can skip
      // computing expensive inputs (e.g. M̂) that an equation never touches.
      const uses = compile(slot.seed, Object.keys(slot.vars)).uses;
      this.entries.set(slot.id, { slot, text: slot.seed, fn: slot.reference, uses, seedUses: uses, status: { state: 'reference', maxDiff: 0, message: '' } });
    }
    this.version = 0; // bumps on every change so callers can cache derived data
  }

  has(id) { return this.entries.has(id); }
  text(id) { return this.entries.get(id).text; }
  status(id) { return this.entries.get(id).status; }
  fn(id) { return this.entries.get(id).fn; }
  uses(id) { return this.entries.get(id).uses; }
  isEdited(id) { return this.entries.get(id).status.state !== 'reference'; }
  editedIds() { return [...this.entries.keys()].filter((id) => this.isEdited(id)); }

  // Try to install new text for a slot. Never throws: a bad edit is reported
  // through the returned status and the previously working version stays.
  set(id, text) {
    const entry = this.entries.get(id);
    const slot = entry.slot;
    if (text.trim() === slot.seed.trim()) return this.reset(id);
    try {
      const compiled = compile(text, Object.keys(slot.vars));
      const check = shapeChecker(slot, this.N);
      const fn = (scope) => check(compiled.evaluate(scope));
      fn.uses = compiled.uses;
      // Validate on random inputs: catches size errors now instead of in the
      // middle of a simulation step, and measures the distance from the reference.
      const random = makeRandom(12345);
      let worst = 0, ref = 0;
      for (let k = 0; k < 24; k++) {
        const sample = slot.sample(random, this.N);
        const got = fn(sample), want = slot.reference(sample);
        worst = Math.max(worst, maxDiff(got, want));
        ref = Math.max(ref, scale(want));
        if (!Number.isFinite(worst) && worst !== Infinity) throw new EquationError('result is not a finite number');
      }
      const matches = worst <= MATCH_TOLERANCE * (1 + ref);
      entry.text = text;
      entry.fn = fn;
      entry.uses = compiled.uses;
      entry.status = {
        state: matches ? 'match' : 'differs', maxDiff: worst,
        message: matches ? 'matches the reference on 24 random inputs' : `differs from the reference (largest difference ${worst.toPrecision(3)} on random inputs)`,
      };
    } catch (e) {
      entry.pendingText = text;
      entry.status = {
        state: 'error', maxDiff: NaN, position: e.position ?? null,
        message: `${e instanceof EquationError ? e.message : `could not evaluate: ${e.message}`} — still running the last working version`,
      };
      return entry.status;
    }
    this.version++;
    return entry.status;
  }

  reset(id) {
    const entry = this.entries.get(id);
    entry.text = entry.slot.seed;
    entry.fn = entry.slot.reference;
    entry.uses = entry.seedUses;
    entry.status = { state: 'reference', maxDiff: 0, message: '' };
    this.version++;
    return entry.status;
  }

  resetAll() { for (const id of this.entries.keys()) this.reset(id); }

  // The compiled seed text (NOT the built-in fast path). Used by the tests to
  // prove that the text shown in the editor really is the reference equation.
  compileSeed(id) {
    const slot = this.entries.get(id).slot;
    const compiled = compile(slot.seed, Object.keys(slot.vars));
    const check = shapeChecker(slot, this.N);
    return (scope) => check(compiled.evaluate(scope));
  }

  // The five RNE equations in the shape rne() expects.
  rneEquations() {
    if (this._rneVersion !== this.version) {
      this._rne = {
        baseAccel: this.fn('rne.baseAccel'), twist: this.fn('rne.twist'), accel: this.fn('rne.accel'),
        wrench: this.fn('rne.wrench'), torque: this.fn('rne.torque'),
      };
      this._rneVersion = this.version;
    }
    return this._rne;
  }

  // (I, m) -> G for buildRobot().
  inertiaFunction() {
    const fn = this.fn('inertia');
    return (I, m) => fn({ I, m });
  }

  // Only edited slots are saved.
  toJSON() {
    const out = {};
    for (const [id, entry] of this.entries) if (entry.status.state !== 'reference') out[id] = entry.text;
    return out;
  }

  load(saved) {
    this.resetAll();
    for (const [id, text] of Object.entries(saved ?? {})) {
      if (this.entries.has(id) && typeof text === 'string') this.set(id, text);
    }
  }
}

// Random-input sampler exposed for the tests.
export { makeRandom };
