// simulation.js — the simulator: true plant + controller + integrator.
//
// This file has no UI code; it runs the same in the browser and in Node
// (the tests drive it headlessly).
//
// Two dynamics models live side by side:
//   plant — the "true" robot. Its equations decide how the arm really moves.
//   hat   — the controller's model of the robot (M̂, ĉ, ĝ). It can be made
//           wrong on purpose (scaled masses, dropped terms, edited equations)
//           to see how each controller copes.
//
// State integrated by RK4:  x = [ θ (N), θ̇ (N), ∫e dt (N), W (1) ]
// where W is the net work done on the arm (motors − friction − work delivered
// to the outside world). Energy conservation says E(t) − E(0) − W(t) = 0 at
// all times, under any controller. That "energy balance" is the most
// sensitive live check that the plant equations are physically right.

import { allFinite, dot, solve, zerosVec } from '../math/linalg.js';
import { fkSpace, jacobianBody, jacobianSpace, pointJacobian } from '../dynamics/kinematics.js';
import { linkKinematics, rne } from '../dynamics/rne.js';
import { coriolisTorques, energy, externalTorques, gravityTorques, massMatrix } from '../dynamics/terms.js';
import { buildRobot, cloneSpec, specProblems } from '../robot/model.js';
import { DEFAULT_PRESET, PRESETS } from '../robot/presets.js';
import { EquationSet } from '../equations/registry.js';
import { CONTROLLER_BY_ID, controlTorque, gainsFromBandwidth } from '../control/controllers.js';
import { makeTrajectory, trajectorySpan } from '../control/trajectory.js';
import { appliedToNotes, pointEllipsoids, reciprocalBasis, splitJointWrench } from '../wrench/wrench.js';
import { rpToTrans, transInv } from '../math/lie.js';
import { rk4Step } from './integrator.js';

const DEG = Math.PI / 180;

export function defaultConfig() {
  return {
    spec: PRESETS[DEFAULT_PRESET](),
    dt: 0.001,                       // RK4 step, s
    controlRate: 0,                  // 0 = controller evaluated continuously; >0 = sample-and-hold at this rate, Hz
    controller: 'ct',
    gains: { mode: 'bandwidth', wn: 8, zeta: 1, Kp: [64, 64, 64], Kd: [16, 16, 16], Ki: [0, 0, 0] },
    trajectory: {
      type: 'quintic',
      start: [0, 20 * DEG, -40 * DEG], end: [70 * DEG, 55 * DEG, -95 * DEG], T: 2.0, repeat: true, dwell: 0.6,
      center: [0, 35 * DEG, -60 * DEG], amplitude: [40 * DEG, 20 * DEG, 30 * DEG], freq: 0.4,
    },
    initial: { theta: [0, 20 * DEG, -40 * DEG], dtheta: [0, 0, 0] },
    // The controller's model. massScale multiplies every link mass (and with
    // it the inertia): 1 = perfect model, 0.8 = controller thinks links are 20% lighter.
    hat: { massScale: 1, dropG: false, dropC: false, knowsWrench: true },
    // Applied wrenches. link: 0…N−1 = that link, N = the tool frame.
    // frame: 'space' ({0}), 'body' (the target's own frame), or 'aligned'
    // (origin at the target frame, axes parallel to {0}).
    wrenches: [],
    wrenchSense: 'applied',          // 'applied' (push ON the link) or 'notes' (wrench the link exerts on the world)
    prediction: { horizon: 1.5, dt: 0.01 },
    logDt: 0.01,
  };
}

// The acceleration a dynamics model predicts for given torques.
//   model: { robot, eqs, gravityOn, coriolisOn, wrenchOn }
function modelAcceleration(model, theta, dtheta, tau, Fext, friction) {
  const robot = model.robot;
  const kin = linkKinematics(robot, theta);
  const Mq = massMatrix(robot, kin, model.eqs);
  const zero = zerosVec(robot.N);
  // bias = everything in τ = M θ̈ + bias that does not multiply θ̈
  const bias = rne(robot, kin, model.coriolisOn ? dtheta : zero, zero, {
    gravity: model.gravityOn ? robot.gravity : null,
    Fext: model.wrenchOn ? Fext : null,
    eqs: model.eqs,
  }).tau;
  const rhs = tau.map((t, i) => t - bias[i] - (friction ? friction[i] : 0));
  const ddtheta = solve(Mq, rhs);
  if (ddtheta === null) throw new Error('mass matrix is singular (an edited equation has probably broken the model)');
  return ddtheta;
}

// Overlay `patch` on `base`: plain objects merge key by key, everything else
// (numbers, strings, arrays) is replaced.
export function mergeConfig(base, patch) {
  const out = { ...base };
  for (const [k, v] of Object.entries(patch ?? {})) {
    const both = v && typeof v === 'object' && !Array.isArray(v) && base[k] && typeof base[k] === 'object' && !Array.isArray(base[k]);
    out[k] = both ? mergeConfig(base[k], v) : cloneSpec(v);
  }
  return out;
}

export class Simulation {
  constructor(config = {}) {
    this.config = mergeConfig(defaultConfig(), config);
    const N = this.config.spec.joints.length;
    this.equations = {
      plant: new EquationSet(['model', 'plant'], N),   // true robot
      model: new EquationSet(['model'], N),            // controller's model
      shared: new EquationSet(['shared'], N),          // control laws, trajectories, wrench tools
    };
    this.fault = null;
    this.rebuild();
    this.reset();
  }

  // ---- configuration ----

  // Recompute everything derived from the configuration and the equations.
  // Cheap; call after any change (the UI calls it through `touch()`).
  rebuild() {
    const cfg = this.config;
    const eq = this.equations;
    this.robot = buildRobot(cfg.spec, eq.plant.inertiaFunction());
    this.N = this.robot.N;

    const hatSpec = cloneSpec(cfg.spec);
    for (const link of hatSpec.links) {
      link.mass *= cfg.hat.massScale;
      if (link.shape.type === 'matrix') link.shape.I = link.shape.I.map((r) => r.map((v) => v * cfg.hat.massScale));
    }
    this.hatRobot = buildRobot(hatSpec, eq.model.inertiaFunction());
    this.hatSpec = hatSpec;

    this.plant = { robot: this.robot, eqs: eq.plant.rneEquations(), gravityOn: true, coriolisOn: true, wrenchOn: true };
    this.hat = {
      robot: this.hatRobot, eqs: eq.model.rneEquations(),
      gravityOn: !cfg.hat.dropG, coriolisOn: !cfg.hat.dropC, wrenchOn: cfg.hat.knowsWrench,
    };

    if (cfg.gains.mode === 'bandwidth') {
      const g = gainsFromBandwidth(cfg.gains.wn, cfg.gains.zeta, this.N);
      cfg.gains.Kp = g.Kp;
      cfg.gains.Kd = g.Kd;
    }
    this.reference = makeTrajectory(cfg.trajectory, { shared: eq.shared });
    this.controller = CONTROLLER_BY_ID[cfg.controller] ?? CONTROLLER_BY_ID.zero;
    this._versions = this.equationVersions();
    this.generation = (this.generation ?? 0) + 1; // lets the UI know derived data changed
  }

  equationVersions() {
    const e = this.equations;
    return `${e.plant.version}/${e.model.version}/${e.shared.version}`;
  }

  touch() {
    this.rebuild();
  }

  // Replace the robot description. Returns a list of problems; if it is not
  // empty nothing was changed.
  applySpec(spec) {
    const problems = specProblems(spec);
    if (problems.length) return problems;
    if (spec.joints.length !== this.N) return ['changing the number of joints requires a new Simulation'];
    this.config.spec = cloneSpec(spec);
    this.rebuild();
    return [];
  }

  // ---- state ----

  reset() {
    const { theta, dtheta } = this.config.initial;
    this.t = 0;
    this.x = [...theta, ...dtheta, ...zerosVec(this.N), 0];
    this.fault = null;
    this.heldTau = null;
    this.nextControlTime = 0;
    this.E0 = energy(this.robot, theta, dtheta).E;
    this.log = [];
    this.nextLogTime = 0;
    this.stepCount = 0;
  }

  get theta() { return this.x.slice(0, this.N); }
  get dtheta() { return this.x.slice(this.N, 2 * this.N); }
  get errorIntegral() { return this.x.slice(2 * this.N, 3 * this.N); }
  get work() { return this.x[3 * this.N]; }

  // ---- wrenches ----

  // All active wrenches as the engine wants them: an array of N+1 entries in
  // {0}, notes' sense (wrench the link exerts on the world), or null if none.
  // `T0` are the current link/tool poses, needed for wrenches typed in a
  // moving frame.
  externalWrenches(T0) {
    const list = this.config.wrenches;
    if (!list.length) return null;
    const transform = this.equations.shared.fn('wrench.transform');
    let Fext = null;
    for (const w of list) {
      if (!w.enabled) continue;
      const F0 = this.wrenchInSpace(w, T0, transform);
      const notes = this.config.wrenchSense === 'applied' ? appliedToNotes(F0) : F0;
      if (!Fext) Fext = new Array(this.N + 1).fill(null);
      Fext[w.link] = Fext[w.link] ? Fext[w.link].map((v, k) => v + notes[k]) : notes;
    }
    return Fext;
  }

  // Pose in {0} of the frame a wrench is typed in.
  wrenchFrame(w, T0) {
    if (w.frame === 'space') return rpToTrans([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0]);
    const T = T0[w.link];
    if (w.frame === 'aligned') return rpToTrans([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [T[0][3], T[1][3], T[2][3]]);
    return T; // 'body'
  }

  // The wrench's six numbers moved into {0}:  F_0 = Ad_{T_f0}ᵀ F_f  (Topic 10 p. 7)
  wrenchInSpace(w, T0, transform = this.equations.shared.fn('wrench.transform')) {
    if (w.frame === 'space') return w.W.slice();
    return transform({ Tab: transInv(this.wrenchFrame(w, T0)), Fa: w.W });
  }

  // ---- dynamics ----

  // Control torque at a given time and state.
  controlAt(t, theta, dtheta, ie, Fext) {
    const shared = this.equations.shared;
    const slot = this.controller.slot;
    const g = this.config.gains;
    return controlTorque(shared.fn(slot), shared.uses(slot), this.hat, {
      t, theta, dtheta, ie, ref: this.reference(t), Kp: g.Kp, Kd: g.Kd, Ki: g.Ki, Fext,
    });
  }

  // ẋ = f(t, x). `model` is the plant to integrate: the true plant for the
  // real simulation, or the controller's model for the "belief" prediction.
  derivative(t, x, model = this.plant, held = null) {
    const N = this.N;
    const theta = x.slice(0, N), dtheta = x.slice(N, 2 * N), ie = x.slice(2 * N, 3 * N);
    const isTrue = model === this.plant;

    let Fext = null;
    if (this.config.wrenches.length) Fext = this.externalWrenches(linkKinematics(this.robot, theta).T0);

    const tau = held ?? this.controlAt(t, theta, dtheta, ie, Fext);
    const friction = isTrue ? this.equations.plant.fn('friction')({ b: this.robot.damping, dtheta }) : null;
    const ddtheta = modelAcceleration(model, theta, dtheta, tau, Fext, friction);

    const ref = this.reference(t);
    const out = new Array(3 * N + 1);
    let powerIn = 0;
    for (let i = 0; i < N; i++) {
      out[i] = dtheta[i];
      out[N + i] = ddtheta[i];
      out[2 * N + i] = ref.theta[i] - theta[i];
      powerIn += (tau[i] - (friction ? friction[i] : 0)) * dtheta[i];
    }
    if (Fext && model.wrenchOn) powerIn -= dot(externalTorques(model.robot, theta, Fext), dtheta);
    out[3 * N] = powerIn;
    return out;
  }

  // Advance the true simulation by n RK4 steps. Returns false if it stopped
  // on a fault (a broken equation or a diverging state); the fault message is
  // in `this.fault` and reset() clears it. It never throws.
  step(n = 1) {
    if (this.fault) return false;
    if (this._versions !== this.equationVersions()) this.rebuild();
    const dt = this.config.dt;
    try {
      for (let k = 0; k < n; k++) {
        let held = null;
        if (this.config.controlRate > 0) {
          if (this.t >= this.nextControlTime - 1e-12 || !this.heldTau) {
            const N = this.N, x = this.x;
            const theta = x.slice(0, N);
            const Fext = this.config.wrenches.length ? this.externalWrenches(linkKinematics(this.robot, theta).T0) : null;
            this.heldTau = this.controlAt(this.t, theta, x.slice(N, 2 * N), x.slice(2 * N, 3 * N), Fext);
            this.nextControlTime = this.t + 1 / this.config.controlRate;
          }
          held = this.heldTau;
        }
        if (this.t >= this.nextLogTime - 1e-12) {
          this.record();
          this.nextLogTime += this.config.logDt;
        }
        this.x = rk4Step((t, x) => this.derivative(t, x, this.plant, held), this.t, this.x, dt);
        this.t += dt;
        this.stepCount++;
        if (!allFinite(this.x) || Math.max(...this.x.slice(this.N, 2 * this.N).map(Math.abs)) > 1e4) {
          throw new Error('the state diverged (joint speed above 10 000 rad/s or not a number)');
        }
      }
    } catch (e) {
      this.fault = { message: e.message, time: this.t };
      return false;
    }
    return true;
  }

  // ---- prediction ----

  // Integrate forward from the current state without touching it, and return
  // the tool-point path.  plant: 'true' -> what will actually happen;
  // 'belief' -> what the controller's own model expects to happen. The gap
  // between the two is the model error, visible before it plays out.
  predict({ plant = 'true', horizon = this.config.prediction.horizon, dt = this.config.prediction.dt } = {}) {
    const model = plant === 'belief' ? this.hat : this.plant;
    const steps = Math.max(1, Math.round(horizon / dt));
    const points = [], thetas = [], times = [];
    let x = this.x.slice(), t = this.t;
    const tool = (th) => { const T = fkSpace(this.robot, th); return [T[0][3], T[1][3], T[2][3]]; };
    const f = (tt, xx) => this.derivative(tt, xx, model, null);
    try {
      for (let k = 0; k <= steps; k++) {
        const th = x.slice(0, this.N);
        points.push(tool(th));
        thetas.push(th);
        times.push(t);
        if (k === steps) break;
        x = rk4Step(f, t, x, dt);
        t += dt;
        if (!allFinite(x)) break;
      }
    } catch (e) {
      // a broken equation: return what we have
    }
    return { points, thetas, times };
  }

  // Tool-point path of the reference over one pass of the trajectory,
  // starting now.
  desiredPath(samples = 160) {
    const span = Math.max(trajectorySpan(this.config.trajectory), 1e-3);
    const points = [];
    for (let k = 0; k <= samples; k++) {
      const T = fkSpace(this.robot, this.reference(this.t + (span * k) / samples).theta);
      points.push([T[0][3], T[1][3], T[2][3]]);
    }
    return points;
  }

  // ---- everything worth displaying at the current state ----

  snapshot() {
    const N = this.N, robot = this.robot;
    const theta = this.theta, dtheta = this.dtheta, ie = this.errorIntegral;
    const kin = linkKinematics(robot, theta);
    const eqs = this.plant.eqs;
    const Fext = this.config.wrenches.length ? this.externalWrenches(kin.T0) : null;
    const ref = this.reference(this.t);
    const zero = zerosVec(N);

    let tau = zero, ddtheta = zero, fault = this.fault;
    const friction = this.equations.plant.fn('friction')({ b: robot.damping, dtheta });
    try {
      tau = this.config.controlRate > 0 && this.heldTau ? this.heldTau : this.controlAt(this.t, theta, dtheta, ie, Fext);
      ddtheta = modelAcceleration(this.plant, theta, dtheta, tau, Fext, friction);
    } catch (e) {
      fault = fault ?? { message: e.message, time: this.t };
    }

    // τ = M θ̈ + c + g + τ_ext + τ_fric : the contributions plotted per joint
    const Mq = massMatrix(robot, kin, eqs);
    const c = coriolisTorques(robot, kin, dtheta, eqs);
    const g = gravityTorques(robot, kin, eqs);
    const ext = externalTorques(robot, theta, Fext);
    const inertial = Mq.map((row) => dot(row, ddtheta));

    // joint wrenches from the backward pass at the actual acceleration
    const pass = rne(robot, kin, dtheta, ddtheta, { gravity: robot.gravity, Fext, eqs });
    const joints = pass.F.map((F, i) => splitJointWrench(F, robot.A[i]));

    const Ttool = kin.T0[N];
    const p = [Ttool[0][3], Ttool[1][3], Ttool[2][3]];
    const Js = jacobianSpace(robot, theta);
    const Jp = pointJacobian(Js, p);
    const en = energy(robot, theta, dtheta, kin);

    return {
      t: this.t, theta, dtheta, ddtheta, ref,
      e: ref.theta.map((r, i) => r - theta[i]),
      tau, parts: { inertial, c, g, ext, friction },
      Mq, kin, T0: kin.T0, Ttool, p, Js, Jp, Fext,
      linkTwists: pass.V, jointWrenches: pass.F, joints,
      energy: en, energyBalance: en.E - this.E0 - this.work,
      fault,
    };
  }

  // Heavier quantities that only some panels need.
  analysis(snap = this.snapshot()) {
    const hatKin = linkKinematics(this.hatRobot, snap.theta);
    return {
      Jb: jacobianBody(this.robot, snap.theta),
      ellipsoids: pointEllipsoids(snap.Jp),
      reciprocal: reciprocalBasis(snap.Js),
      Mhat: massMatrix(this.hatRobot, hatKin, this.hat.eqs),
      chat: this.hat.coriolisOn ? coriolisTorques(this.hatRobot, hatKin, snap.dtheta, this.hat.eqs) : zerosVec(this.N),
      ghat: this.hat.gravityOn ? gravityTorques(this.hatRobot, hatKin, this.hat.eqs) : zerosVec(this.N),
    };
  }

  // Append the current state to the plot history (bounded length).
  record() {
    const s = this.snapshot();
    this.log.push({
      t: s.t, theta: s.theta, thetad: s.ref.theta, e: s.e, tau: s.tau,
      inertial: s.parts.inertial, c: s.parts.c, g: s.parts.g, ext: s.parts.ext, friction: s.parts.friction,
      K: s.energy.K, P: s.energy.P, E: s.energy.E, balance: s.energyBalance, p: s.p,
      bearingForce: s.joints.map((j) => j.forceMagnitude), bearingMoment: s.joints.map((j) => j.bendingMoment),
    });
    if (this.log.length > 4000) this.log.splice(0, this.log.length - 4000);
  }

  // ---- save / load ----

  toJSON() {
    return {
      format: 'robot-dynamics-sim/1',
      config: cloneSpec(this.config),
      equations: {
        plant: this.equations.plant.toJSON(),
        model: this.equations.model.toJSON(),
        shared: this.equations.shared.toJSON(),
      },
    };
  }

  // Returns a list of problems; if it is not empty nothing was changed.
  load(saved) {
    if (!saved || saved.format !== 'robot-dynamics-sim/1' || !saved.config) return ['not a robot-dynamics-sim setup file'];
    const cfg = mergeConfig(defaultConfig(), saved.config);
    const problems = specProblems(cfg.spec);
    if (cfg.spec.joints?.length !== this.N) problems.push('setup has a different number of joints');
    if (!(cfg.dt > 0 && cfg.dt <= 0.05)) problems.push('time step must be between 0 and 0.05 s');
    if (problems.length) return problems;
    this.config = cfg;
    this.equations.plant.load(saved.equations?.plant);
    this.equations.model.load(saved.equations?.model);
    this.equations.shared.load(saved.equations?.shared);
    this.rebuild();
    this.reset();
    return [];
  }
}
