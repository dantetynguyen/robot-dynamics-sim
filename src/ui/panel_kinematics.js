// panel_kinematics.js — Kinematics mode: no physics, just geometry.
// Drag the joints and see the screw axes, the product of exponentials, the
// Jacobians and the twist they produce.

import { fkSpace, jacobianBody, jacobianSpace, pointJacobian } from '../dynamics/kinematics.js';
import { matVec } from '../math/linalg.js';
import { pointEllipsoids } from '../wrench/wrench.js';
import { button, checkbox, el, field, section, slider } from './dom.js';
import { fmt, fmtVec, matrixTable, tex } from './format.js';

export function kinematicsPanel(app) {
  const kin = app.view.kin, robot = app.sim.robot, N = robot.N, show = app.view.show;
  const root = el('div');
  const lim = app.toUI(Math.PI);
  const posSliders = kin.theta.map((t, i) => slider({
    min: -lim, max: lim, step: app.view.units === 'deg' ? 1 : 0.01, value: app.toUI(t),
    format: (v) => `${fmt(v, app.view.units === 'deg' ? 0 : 2)} ${app.angleUnit}`, onInput: (v) => { kin.theta[i] = app.fromUI(v); app.save(); },
  }));
  const setPose = (theta) => { theta.forEach((t, i) => { kin.theta[i] = t; posSliders[i].set(app.toUI(t)); }); app.save(); };
  const toggle = (key, label) => checkbox(label, show[key], (v) => { show[key] = v; app.renderLegend(); app.save(); });

  root.append(
    section('Joint angles θ', true,
      kin.theta.map((_, i) => field(`θ${i + 1}`, posSliders[i])),
      el('div', { class: 'row' },
        button('Home (θ = 0)', () => setPose(new Array(N).fill(0)), 'small'),
        button('Bent pose', () => setPose([0.4, 0.6, -0.9]), 'small'),
        button('Nearly straight elbow', () => setPose([0.4, 0.5, -0.04]), 'small'),
        button('Copy from the simulation', () => setPose(app.sim.theta), 'small'))),
    section('Joint rates θ̇', true,
      kin.dtheta.map((d, i) => field(`θ̇${i + 1}`, slider({ min: -2, max: 2, step: 0.05, value: d, format: (v) => `${fmt(v, 2)} rad/s`, onInput: (v) => { kin.dtheta[i] = v; app.save(); } })))),
    section('Show', true, el('div', { class: 'checks-grid' },
      toggle('axes', 'Screw axes S₁…S₃'), toggle('poe', 'PoE build-up'), toggle('jacobian', 'Tool velocity by joint'),
      toggle('ellipsoids', 'Ellipsoids at the tool'), toggle('frames', 'Link frames'))),
  );

  const fkEl = el('div'), jsEl = el('div'), jbEl = el('div'), twEl = el('div'), ellEl = el('div');
  root.append(
    section('Forward kinematics: product of exponentials', true,
      tex('T(\\theta)=e^{[\\mathcal S_1]\\theta_1}\\,e^{[\\mathcal S_2]\\theta_2}\\,e^{[\\mathcal S_3]\\theta_3}\\,M', true), fkEl,
      el('p', { class: 'note' }, 'Each screw axis S_i = (ŝ, −ŝ×q) is written once, in the space frame, with the arm at home. Turn on "PoE build-up" to watch the pose assemble: M first, then joint 3\'s exponential, then joint 2\'s, then joint 1\'s. A joint nearer the base moves everything after it.'),
      el('p', { class: 'dh' }, 'This is the same pose a Denavit–Hartenberg chain ⁰A₁ ¹A₂ ²A₃ gives. DH needs a frame on every link placed by its rules; PoE needs only {0}, the tool frame and the joint axes.'),
      el('div', { class: 'cite' }, 'Topic 4 (screw axes, PoE); MR Eq. 4.14, with e^{[S]θ} from MR Eqs. 3.51 and 3.88')),
    section('Space Jacobian J_s', true,
      tex('V_s = J_s(\\theta)\\,\\dot\\theta,\\qquad J_{s,i}=\\mathrm{Ad}_{e^{[\\mathcal S_1]\\theta_1}\\cdots e^{[\\mathcal S_{i-1}]\\theta_{i-1}}}\\,\\mathcal S_i', true), jsEl,
      el('p', { class: 'note' }, 'Column i is joint i\'s screw axis where it is right now, seen from {0}. Turn on "Screw axes": the dashed lines are the columns. Column 1 never changes; column 3 moves when joints 1 or 2 move.'),
      el('p', { class: 'dh' }, 'A DH Jacobian column is (ẑ × (o_n − o), ẑ): linear part first, measured at the tool point. The screw column is (ŝ, −ŝ × q): angular part first, measured at the {0} origin. Same joint axis, different bookkeeping.'),
      el('div', { class: 'cite' }, 'Topic 5 (adjoint, space Jacobian); MR Eq. 5.11')),
    section('Body Jacobian J_b', false,
      tex('V_b = J_b(\\theta)\\,\\dot\\theta,\\qquad J_b = \\mathrm{Ad}_{T^{-1}}\\,J_s', true), jbEl,
      el('p', { class: 'note' }, 'The same twist expressed in the tool frame. The adjoint is the 6×6 that changes the frame a twist is written in.'),
      el('div', { class: 'cite' }, 'Topic 6 (body Jacobian); MR Eqs. 5.18 and 5.22')),
    section('Twist of the tool', true, twEl,
      el('p', { class: 'note' }, 'The arrows at the tool are each joint\'s share of the tool-point velocity, ṗ = v_s + ω_s × p, and their sum. The dashed line is the screw axis of V_s: at this instant the tool frame is turning about that line.'),
      el('div', { class: 'cite' }, 'Topic 4 (twists as screws), Topic 5; MR Def. 5.1')),
    section('Manipulability and singularities', true, ellEl,
      el('p', { class: 'note' }, 'The velocity ellipsoid is the set of tool-point velocities reachable with ‖θ̇‖ = 1; its axes come from the eigenvectors and eigenvalues of J Jᵀ. Straighten the elbow: one axis collapses (a direction the tool cannot move in) and the force ellipsoid stretches along it (a direction it can resist with almost no torque).'),
      el('div', { class: 'cite' }, 'Topic 7 (singularities, manipulability measures μ₁, μ₂, μ₃); Topic 10 pp. 8–9; MR §5.3–5.4')),
  );

  const refresh = () => {
    const theta = kin.theta, dtheta = kin.dtheta;
    const T = fkSpace(robot, theta), Js = jacobianSpace(robot, theta), Jb = jacobianBody(robot, theta);
    const p = [T[0][3], T[1][3], T[2][3]], Jp = pointJacobian(Js, p);
    const Vs = matVec(Js, dtheta), Vb = matVec(Jb, dtheta), pd = matVec(Jp, dtheta);
    const e = pointEllipsoids(Jp);
    fkEl.replaceChildren(el('div', { class: 'matrow' }, 'T(θ) =', matrixTable(T, 3)));
    jsEl.replaceChildren(el('div', { class: 'matrow' }, 'J_s =', matrixTable(Js, 3)));
    jbEl.replaceChildren(el('div', { class: 'matrow' }, 'J_b =', matrixTable(Jb, 3)));
    twEl.replaceChildren(el('dl', { class: 'kv' },
      el('dt', {}, 'V_s = (ω, v)'), el('dd', {}, fmtVec(Vs, 3)),
      el('dt', {}, 'V_b = (ω, v)'), el('dd', {}, fmtVec(Vb, 3)),
      el('dt', {}, 'tool-point velocity ṗ'), el('dd', {}, `${fmtVec(pd, 3)} m/s`)));
    const near = e.mu1 > 25;
    ellEl.replaceChildren(el('dl', { class: 'kv' },
      el('dt', {}, 'velocity semi-axes'), el('dd', {}, `${fmtVec(e.manipulability, 3)} m/s per rad/s`),
      el('dt', {}, 'force semi-axes'), el('dd', {}, `${fmtVec(e.force, 2)} N per N·m`),
      el('dt', {}, 'μ₁ = √(λmax/λmin)'), el('dd', {}, fmt(e.mu1, 2)),
      el('dt', {}, 'μ₂ = λmax/λmin'), el('dd', {}, fmt(e.mu2, 1)),
      el('dt', {}, 'μ₃ = √det(JJᵀ)'), el('dd', {}, fmt(e.mu3, 5))),
      el('p', { class: `status-line ${near ? 'warn' : 'ok'}` }, near ? 'Close to a singularity: one velocity axis has nearly collapsed.' : 'Well away from a singularity.'));
  };
  refresh();
  return { el: root, refresh };
}
