// experiments.js — the guided "Try this" experiments.
//
// TO ADD AN EXPERIMENT: append an object to EXPERIMENTS.
//   mode     'dynamics' or 'kinematics'
//   config   a partial Simulation config, merged over the defaults
//   view     partial view state (tab, which chart, which overlays)
//   after    optional function(app) run once the setup is loaded (e.g. to edit
//            an equation or add a wrench that depends on the pose)
//   actions  optional buttons shown on the card: [{ label, run(app) }]
// The text fields say what to do, what to watch, and why it happens.

import { SLOT_BY_ID } from '../equations/registry.js';
import { reciprocalBasis } from '../wrench/wrench.js';

const D = Math.PI / 180;
const HOLD = [0, 35 * D, -70 * D];
const hold = (pose = HOLD) => ({ trajectory: { type: 'hold', start: pose }, initial: { theta: pose, dtheta: [0, 0, 0] } });
const swing = { controller: 'zero', initial: { theta: [0.3, 60 * D, -30 * D], dtheta: [1.5, 0, 0] }, trajectory: { type: 'hold', start: [0.3, 60 * D, -30 * D] } };
const quiet = { desired: false, belief: false, ghosts: false };

export const EXPERIMENTS = [
  {
    id: 'energy', mode: 'dynamics', title: 'Motors off: energy is conserved',
    concept: 'Forward dynamics and time integration',
    config: swing, view: { chart4: 'energy', show: quiet },
    todo: 'Press Run. The arm falls and tumbles with zero joint torque and no friction.',
    watch: 'Energy chart: kinetic and potential trade back and forth while the total stays flat, and the dashed balance line stays on zero. Then raise the time step to 10 ms (Control → Simulation) and watch the total start to drift.',
    why: 'With τ = 0 nothing does work on the arm, so K + P cannot change. Any drift is integration error. RK4 at 1 ms keeps it near 10⁻⁶ J over 10 s.',
    cite: 'Topic 13 pp. 7–8; MR §8.5',
  },
  {
    id: 'ct', mode: 'dynamics', title: 'Computed torque with a perfect model',
    concept: 'Feedback linearisation gives second-order error dynamics',
    config: { controller: 'ct', gains: { mode: 'bandwidth', wn: 8, zeta: 1 }, initial: { theta: [0.3, 20 * D + 0.25, -40 * D - 0.3], dtheta: [0, 0, 0] } },
    view: { chart4: 'energy' },
    todo: 'Press Run. The arm starts away from the reference and catches up while the reference keeps moving.',
    watch: 'Tracking-error chart: all three joints decay with the same shape, even though the arm\'s inertia is changing. Now set ζ = 0.3 and press Reset: the same three curves overshoot and ring. Predicted and belief paths lie on top of each other.',
    why: 'The law cancels M, c and g with the model, leaving ë + K_d ė + K_p e = 0 on every joint: a mass–spring–damper you choose with ωₙ and ζ.',
    cite: 'MR Eqs. 11.8, 11.33, 11.37',
  },
  {
    id: 'droop', mode: 'dynamics', title: 'Forget gravity in the model: the arm droops',
    concept: 'What the ĝ term is for',
    config: { controller: 'ct', hat: { dropG: true }, ...hold() }, view: { chart4: 'energy' },
    todo: 'Press Run. The controller is asked to hold a pose, but its model has no gravity.',
    watch: 'The arm sags to a steady error where the "spring" K_p e balances the weight. The violet belief path stays put at the target: the controller expects no sag. Raise ωₙ and the droop shrinks but never reaches zero. Untick "Drop ĝ" and it vanishes.',
    why: 'At rest the true plant needs τ = g(θ). Without ĝ the only source of torque is feedback, and feedback needs an error to produce it.',
    cite: 'MR Eq. 11.38 and the discussion after it',
  },
  {
    id: 'modelerr', mode: 'dynamics', title: 'A wrong model: predicted and belief paths split',
    concept: 'Model error made visible before it happens',
    config: { controller: 'ct', hat: { massScale: 0.6 } }, view: {},
    todo: 'Press Run. The controller believes every link is 40% lighter than it is.',
    watch: 'Blue (true plant) and violet (controller\'s belief) leave the tool together and separate: that gap is the model error. Drag "Mass error" back to 0% and they merge. Then compare PD + gravity compensation at the same error.',
    why: 'Computed torque is only as good as M̂, ĉ, ĝ. With M̂ too small the feedforward under-drives every acceleration and the feedback loop has to make up the difference.',
    cite: 'MR §11.4.1.2 (feedforward with an incorrect model), Eq. 11.37',
  },
  {
    id: 'nullwrench', mode: 'dynamics', title: 'Push where the arm cannot feel it',
    concept: 'Reciprocal wrenches: null(J_sᵀ)',
    config: { controller: 'pdg', hat: { knowsWrench: false }, ...hold() }, view: { tab: 'wrench', show: { ...quiet, wrench: true } },
    after: (app) => {
      const s = app.sim.snapshot();
      const b = reciprocalBasis(s.Js).basis[0];
      const scale = 40 / Math.max(Math.hypot(b[3], b[4], b[5]), 0.2);
      app.sim.config.wrenches.push({ enabled: true, link: app.sim.N, frame: 'space', W: b.map((x) => Number((x * scale).toFixed(6))) });
      app.changed();
    },
    todo: 'Press Run. A wrench with a 40 N force is being applied to the tool, and the controller has not been told.',
    watch: 'Nothing moves and the torque chart does not change. Now edit one of its six numbers in the Wrenches tab: the arm is pushed away at once.',
    why: 'This wrench is reciprocal to all three joint screws (J_sᵀF = 0), so it does no work on any joint motion. The links and bearings carry it; the motors never see it.',
    cite: 'Topic 10 pp. 3–5; MR §5.2',
  },
  {
    id: 'threeframes', mode: 'dynamics', title: 'One wrench, three frames, the same torques',
    concept: 'Wrench transformation and frame-independent power',
    config: { controller: 'ct', wrenches: [{ enabled: true, link: 3, frame: 'aligned', W: [0, 0, 0, 0, 0, -15] }] }, view: { tab: 'wrench' },
    todo: 'Press Run and open "The same wrench in three frames" in the Wrenches tab.',
    watch: 'The six numbers differ in every row, but VᵀF is identical in all three and the joint torque it costs is one vector. Switch the wrench\'s frame to "Space frame {0}" and keep the numbers: now it is a different physical wrench (a force through the base origin) and the torques change.',
    why: 'F_b = Ad_{T_ab}ᵀ F_a is exactly the rule that keeps power VᵀF unchanged when the frame changes.',
    cite: 'Topic 10 p. 7; MR Eqs. 3.94–3.95',
  },
  {
    id: 'bearing', mode: 'dynamics', title: 'Spin the waist: the bearings feel it',
    concept: 'Motor torque versus bearing load',
    config: {
      controller: 'ct', trajectory: { type: 'sine', center: [0, 0, 0], amplitude: [170 * D, 0, 0], freq: 0.6 },
      initial: { theta: [0, 0, 0], dtheta: [170 * D * 2 * Math.PI * 0.6, 0, 0] },
    },
    view: { chart4: 'bearing', torqueJoint: 1, show: { ...quiet, bearing: true } },
    todo: 'Press Run. The arm is held straight out while the waist swings back and forth.',
    watch: 'Bearing-force chart: joints 2 and 3 peak when the waist is fastest (centripetal load, ∝ θ̇₁²), although their motors only hold the arm up. Raise the frequency and the peaks grow with its square.',
    why: 'The backward pass gives the whole wrench through each joint. Only the component along the joint screw is motor torque; the centripetal pull along the arm is perpendicular to it and lands on the bearings.',
    cite: 'Topic 10 p. 6; Topic 11 p. 13; MR Eq. 8.54',
  },
  {
    id: 'breakeq', mode: 'dynamics', title: 'Break an equation and read the checks',
    concept: 'Which property does each term protect?',
    config: swing, view: { tab: 'checks', chart4: 'energy', show: { ...quiet, predicted: false } },
    todo: 'Press Run, then try each button and watch the Checks tab and the Energy chart.',
    actions: [
      { label: 'Use the diag([ω],[v]) velocity term', run: (app) => edit(app, 'rne.wrench', SLOT_BY_ID['rne.wrench'].wrong[0].text) },
      { label: 'Delete the Coriolis term from V̇ᵢ', run: (app) => edit(app, 'rne.accel', SLOT_BY_ID['rne.accel'].wrong[0].text) },
      { label: 'Restore both', run: (app) => { edit(app, 'rne.wrench', SLOT_BY_ID['rne.wrench'].seed); edit(app, 'rne.accel', SLOT_BY_ID['rne.accel'].seed); } },
    ],
    watch: 'With the diag([ω],[v]) term the motion is wrong and the Christoffel check fails, yet energy is still conserved. Delete the Coriolis term from the acceleration recursion instead and the energy balance breaks too.',
    why: 'The diag([ω],[v]) form drops the force m ω×v. That force is always perpendicular to v, so it does no work: energy conservation cannot see it. Passing the energy check is necessary, not sufficient.',
    cite: 'Topic 11 pp. 5–6, 13; MR Eqs. 8.29, 8.40, 8.52',
  },
  {
    id: 'modeledit', mode: 'dynamics', title: 'Edit only the controller\'s model',
    concept: 'True plant versus the controller\'s copy',
    config: { controller: 'ct' }, view: { tab: 'equations', eqTarget: 'model' },
    after: (app) => { app.sim.equations.model.set('inertia', 'blkdiag(I, 2*m*eye(3))'); app.changed(); },
    todo: 'Press Run. The controller\'s spatial inertia has been edited to double every link mass; the true plant is untouched.',
    watch: 'Tracking gets worse and blue/violet split, while the Checks tab shows the true plant passing everything and only the controller\'s model failing "M from RNE = M from kinetic energy".',
    why: 'The plant obeys physics no matter what the controller believes. An edit to the model changes only the torques the controller asks for.',
    cite: 'Topic 11 p. 5; MR Eq. 8.32',
  },
  {
    id: 'poe', mode: 'kinematics', title: 'Build the pose one exponential at a time',
    concept: 'Product of exponentials',
    view: { show: { poe: true, axes: true, jacobian: false, ellipsoids: false }, kin: { theta: [0.7, 0.6, -0.9], dtheta: [0, 0, 0] } },
    todo: 'Drag θ₃, then θ₂, then θ₁.',
    watch: 'Three faint arms: home (M), then with joint 3 turned, then with joints 2 and 3 turned. θ₃ moves only the last ghost onward; θ₁ moves only the solid arm. Every screw axis is drawn where it was at home for the ghost it acts on.',
    why: 'T = e^{[S₁]θ₁}(e^{[S₂]θ₂}(e^{[S₃]θ₃} M)). Reading right to left, the tool-most joint acts first, while the others are still at home, so its home axis S₃ is still correct.',
    cite: 'Topic 4; MR Eq. 4.14',
  },
  {
    id: 'jcols', mode: 'kinematics', title: 'Jacobian columns are the joint screws',
    concept: 'Space Jacobian',
    view: { show: { poe: false, axes: true, jacobian: true, ellipsoids: false }, kin: { theta: [0.4, 0.6, -0.9], dtheta: [1, 0, 0] } },
    todo: 'Set one joint rate at a time to a non-zero value, then mix them.',
    watch: 'With only θ̇₁ the tool velocity arrow circles axis S₁; with only θ̇₃ it circles S₃. Mixed, the white arrow is the vector sum of the coloured ones and the dashed line (the screw axis of V_s) moves between the joint axes.',
    why: 'V_s = J_s θ̇ is a weighted sum of the columns, and column i is joint i\'s screw axis in the current pose.',
    cite: 'Topic 5; MR Eq. 5.11',
  },
  {
    id: 'singular', mode: 'kinematics', title: 'Near a singularity the force ellipsoid stretches',
    concept: 'Duality of the velocity and force ellipsoids',
    view: { show: { poe: false, axes: false, jacobian: false, ellipsoids: true }, kin: { theta: [0.4, 0.5, -0.9], dtheta: [0, 0, 0] } },
    todo: 'Drag θ₃ slowly towards 0 (elbow straight).',
    watch: 'The green velocity ellipsoid flattens along the arm while the red force ellipsoid grows along it without limit. μ₁ climbs in the panel.',
    why: 'Both ellipsoids share their axes and their lengths are reciprocals (√λ and 1/√λ of J Jᵀ). The direction the tool can no longer move in is the direction it can resist with almost no torque: a straight arm carries a load along itself for free.',
    cite: 'Topic 7; Topic 10 pp. 8–9 (as corrected in class); MR §5.4',
  },
];

function edit(app, id, text) {
  app.sim.equations.plant.set(id, text);
  app.changed();
  if (app.view.tab === 'equations' || app.view.tab === 'checks') app.renderTab();
}
