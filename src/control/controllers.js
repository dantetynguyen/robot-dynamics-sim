// controllers.js — the selectable controllers.
//
// A controller is a named control LAW, and every law is an editable equation
// (slots 'ctrl.*' in src/equations/registry.js). The law sees the tracking
// error and, if it asks for them, quantities from the controller's own model
// of the robot (M̂, ĉ, ĝ): the "hat" model, which may be deliberately wrong.
//
// TO ADD A CONTROLLER:
//   1. add its law to controllerSlots() in src/equations/registry.js;
//   2. add one line to CONTROLLERS below.
//
// Source for the laws: MR Ch. 11 (the course notes do not cover control).

import { diag, zerosVec } from '../math/linalg.js';
import { linkKinematics } from '../dynamics/rne.js';
import { coriolisTorques, externalTorques, gravityTorques, massMatrix } from '../dynamics/terms.js';

export const CONTROLLERS = [
  { id: 'zero', slot: 'ctrl.zero', name: 'Zero torque' },
  { id: 'pd', slot: 'ctrl.pd', name: 'PD' },
  { id: 'pdg', slot: 'ctrl.pdg', name: 'PD + gravity compensation' },
  { id: 'ct', slot: 'ctrl.ct', name: 'Computed torque' },
];

export const CONTROLLER_BY_ID = Object.fromEntries(CONTROLLERS.map((c) => [c.id, c]));

// Gains from a target closed-loop response. Under computed torque with a
// perfect model each joint error obeys  ë + K_d ė + K_p e = 0  (MR Eq. 11.33
// with K_i = 0). Matching that to the standard second-order form
//   ë + 2ζω_n ė + ω_n² e = 0                                   (MR Eq. 11.8)
// gives K_p = ω_n², K_d = 2ζω_n.
export function gainsFromBandwidth(wn, zeta, N) {
  return {
    Kp: new Array(N).fill(wn * wn),
    Kd: new Array(N).fill(2 * zeta * wn),
  };
}

// Exact solution of ë + 2ζω_n ė + ω_n² e = 0 from e(0) = e0, ė(0) = de0
// (MR §11.2.2, the three cases of Eq. 11.10). Used as the yardstick for the
// computed-torque acceptance test and drawn in the error plot.
export function secondOrderResponse(wn, zeta, e0, de0, t) {
  if (Math.abs(zeta - 1) < 1e-9) {
    // critically damped: (c1 + c2 t) e^{−ω_n t}
    return (e0 + (de0 + wn * e0) * t) * Math.exp(-wn * t);
  }
  if (zeta > 1) {
    // overdamped: two real roots s1, s2
    const r = wn * Math.sqrt(zeta * zeta - 1);
    const s1 = -zeta * wn + r, s2 = -zeta * wn - r;
    const c1 = (de0 - s2 * e0) / (s1 - s2), c2 = e0 - c1;
    return c1 * Math.exp(s1 * t) + c2 * Math.exp(s2 * t);
  }
  // underdamped: decaying oscillation at ω_d = ω_n √(1 − ζ²)
  const wd = wn * Math.sqrt(1 - zeta * zeta);
  return Math.exp(-zeta * wn * t) * (e0 * Math.cos(wd * t) + ((de0 + zeta * wn * e0) / wd) * Math.sin(wd * t));
}

// Evaluate a control law.
//   law      compiled/reference law function, law.uses or `uses` = Set of variable names it reads
//   hat      the controller's dynamics model: { robot, eqs, gravityOn, coriolisOn, wrenchOn }
//   in       { t, theta, dtheta, ie, ref: {theta, dtheta, ddtheta}, Kp, Kd, Ki (arrays), Fext }
// Only the model quantities the law actually mentions are computed.
export function controlTorque(law, uses, hat, input) {
  const { theta, dtheta, ref } = input;
  const N = theta.length;
  const scope = {
    t: input.t,
    theta, dtheta,
    thetad: ref.theta, dthetad: ref.dtheta, ddthetad: ref.ddtheta,
    e: ref.theta.map((r, i) => r - theta[i]),
    de: ref.dtheta.map((r, i) => r - dtheta[i]),
    ie: input.ie,
    Kp: diag(input.Kp), Kd: diag(input.Kd), Ki: diag(input.Ki),
  };
  const needsModel = uses.has('Mhat') || uses.has('chat') || uses.has('ghat');
  if (needsModel) {
    const kin = linkKinematics(hat.robot, theta);
    if (uses.has('Mhat')) scope.Mhat = massMatrix(hat.robot, kin, hat.eqs);
    if (uses.has('chat')) scope.chat = hat.coriolisOn ? coriolisTorques(hat.robot, kin, dtheta, hat.eqs) : zerosVec(N);
    if (uses.has('ghat')) scope.ghat = hat.gravityOn ? gravityTorques(hat.robot, kin, hat.eqs) : zerosVec(N);
  }
  if (uses.has('what')) {
    scope.what = hat.wrenchOn && input.Fext ? externalTorques(hat.robot, theta, input.Fext) : zerosVec(N);
  }
  return law(scope);
}
