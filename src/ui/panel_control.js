// panel_control.js — the Control tab: controller, gains, the controller's
// model, the reference trajectory, simulation and display settings.

import { CONTROLLERS } from '../control/controllers.js';
import { TRAJECTORIES } from '../control/trajectory.js';
import { button, checkbox, el, field, numberInput, section, select, slider, vectorInput } from './dom.js';
import { fmt, fmtVec, tex } from './format.js';
import { setupControls } from './setup_io.js';

export function controlPanel(app) {
  const sim = app.sim, cfg = sim.config, N = sim.N;
  const root = el('div');
  const rerender = () => { app.changed(); app.renderTab(); };
  const angles = (values, onChange, lim = 720) => vectorInput({
    values: values.map((v) => app.toUI(v)), min: -app.toUI(lim * Math.PI / 180), max: app.toUI(lim * Math.PI / 180),
    onChange: (vals) => onChange(vals.map((v) => app.fromUI(v))),
  });

  // ----- controller -----
  const law = sim.equations.shared;
  const slot = sim.controller.slot;
  const gains = cfg.gains;
  const gainRows = gains.mode === 'bandwidth' ? [
    field('Natural frequency ωₙ', slider({ min: 1, max: 40, step: 0.5, value: gains.wn, format: (v) => `${fmt(v, 1)} rad/s`, onInput: (v) => { gains.wn = v; app.changed(); } })),
    field('Damping ratio ζ', slider({ min: 0.05, max: 2, step: 0.05, value: gains.zeta, format: (v) => fmt(v, 2), onInput: (v) => { gains.zeta = v; app.changed(); } }),
      'Kp = ωₙ² and Kd = 2ζωₙ on every joint. ζ = 1 is critically damped: fastest approach with no overshoot.'),
  ] : [
    field('Kp per joint', vectorInput({ values: gains.Kp, min: 0, max: 1e5, onChange: (v) => { gains.Kp = v; app.changed(); } })),
    field('Kd per joint', vectorInput({ values: gains.Kd, min: -1e4, max: 1e4, onChange: (v) => { gains.Kd = v; app.changed(); } }), 'A negative Kd pumps energy in; try it to see the run stop safely.'),
  ];
  root.append(section('Controller', true,
    field('Law', select(CONTROLLERS.map((c) => [c.id, c.name]), cfg.controller, (v) => { cfg.controller = v; rerender(); })),
    el('p', { class: 'note' }, el('code', {}, `τ = ${law.text(slot)}`), law.isEdited(slot) ? ' (edited)' : ''),
    el('p', { class: 'note' }, law.entries.get(slot).slot.note),
    field('Gains from', select([['bandwidth', 'ωₙ and ζ'], ['manual', 'Kp and Kd directly']], gains.mode, (v) => { gains.mode = v; rerender(); })),
    gainRows,
    field('Ki per joint', vectorInput({ values: gains.Ki, min: 0, max: 1e5, onChange: (v) => { gains.Ki = v; app.changed(); } }), 'Integral gain. 0 = off. It removes steady-state error at the cost of overshoot.'),
  ));

  // ----- the controller's model -----
  const hat = cfg.hat;
  root.append(section("Controller's model (M̂, ĉ, ĝ)", true,
    el('p', { class: 'note' }, 'The controller computes torques from its own copy of the robot. Make that copy wrong and watch the predicted path (true plant) split from the belief path (this model).'),
    field('Mass error', slider({ min: 0.4, max: 1.6, step: 0.01, value: hat.massScale, format: (v) => `${v >= 1 ? '+' : '−'}${fmt(Math.abs(v - 1) * 100, 0)}%`, onInput: (v) => { hat.massScale = v; app.changed(); } }),
      'Scales every link mass and inertia in the controller\'s model. 0% = perfect model.'),
    checkbox('Drop ĝ (model ignores gravity)', hat.dropG, (v) => { hat.dropG = v; app.changed(); }),
    checkbox('Drop ĉ (model ignores Coriolis and centripetal torques)', hat.dropC, (v) => { hat.dropC = v; app.changed(); }),
    checkbox('Controller knows the applied wrenches', hat.knowsWrench, (v) => { hat.knowsWrench = v; app.changed(); }),
  ));

  // ----- trajectory -----
  const tr = cfg.trajectory;
  const p2p = tr.type === 'cubic' || tr.type === 'quintic';
  root.append(section('Reference trajectory', true,
    field('Type', select(Object.entries(TRAJECTORIES).map(([id, t]) => [id, t.name]), tr.type, (v) => { tr.type = v; rerender(); })),
    tr.type === 'sine' ? [
      field(`Centre (${app.angleUnit})`, angles(tr.center, (v) => { tr.center = v; app.changed(); })),
      field(`Amplitude (${app.angleUnit})`, angles(tr.amplitude, (v) => { tr.amplitude = v; app.changed(); })),
      field('Frequency', numberInput({ value: tr.freq, min: 0.01, max: 10, onChange: (v) => { tr.freq = v; app.changed(); } }), 'Hz'),
    ] : [
      field(`${p2p ? 'Start' : 'Pose'} (${app.angleUnit})`, angles(tr.start, (v) => { tr.start = v; app.changed(); })),
      p2p && field(`End (${app.angleUnit})`, angles(tr.end, (v) => { tr.end = v; app.changed(); })),
      p2p && field('Duration T', numberInput({ value: tr.T, min: 0.05, max: 60, onChange: (v) => { tr.T = v; app.changed(); } }), 'seconds for one move'),
      p2p && checkbox('Repeat: go there, pause, come back', tr.repeat, (v) => { tr.repeat = v; app.changed(); }),
    ],
    p2p && el('p', { class: 'note' }, tex('\\theta_d(t)=\\theta_{start}+s(t)\\,(\\theta_{end}-\\theta_{start})'), ' with s(t) = ',
      el('code', {}, sim.equations.shared.text(`traj.${tr.type}`)), '. Edit s(t) in the Equations tab.'),
    field(`Initial pose (${app.angleUnit})`, angles(cfg.initial.theta, (v) => { cfg.initial.theta = v; app.changed(); })),
    field(`Initial rates (${app.angleUnit}/s)`, angles(cfg.initial.dtheta, (v) => { cfg.initial.dtheta = v; app.changed(); }, 5000)),
    el('div', { class: 'row' },
      button('Start at the trajectory start', () => { cfg.initial.theta = (tr.type === 'sine' ? tr.center : tr.start).slice(); cfg.initial.dtheta = new Array(N).fill(0); app.changed({ reset: true }); app.renderTab(); }, 'small'),
      button('Apply and reset', () => app.changed({ reset: true }), 'small')),
  ));

  // ----- simulation -----
  root.append(section('Simulation', false,
    field('Time step', numberInput({ value: cfg.dt * 1000, min: 0.05, max: 20, onChange: (v) => { cfg.dt = v / 1000; app.changed(); } }), 'ms, fixed-step RK4. Larger steps run faster and drift more.'),
    field('Control rate', numberInput({ value: cfg.controlRate, min: 0, max: 20000, onChange: (v) => { cfg.controlRate = v; app.changed(); } }), 'Hz. 0 = the law is evaluated continuously. Try 20 Hz to see what a slow digital controller does.'),
    field('Prediction horizon', numberInput({ value: cfg.prediction.horizon, min: 0.1, max: 10, onChange: (v) => { cfg.prediction.horizon = v; app.changed(); } }), 'seconds ahead'),
    field('Prediction step', numberInput({ value: cfg.prediction.dt * 1000, min: 1, max: 50, onChange: (v) => { cfg.prediction.dt = v / 1000; app.changed(); } }), 'ms'),
  ));

  // ----- display -----
  const show = app.view.show;
  const toggle = (key, label) => checkbox(label, show[key], (v) => { show[key] = v; app.predDirty = true; app.renderLegend(); app.save(); });
  root.append(section('Display', false,
    el('div', { class: 'checks-grid' },
      toggle('desired', 'Desired path'), toggle('predicted', 'Predicted path'), toggle('belief', "Controller's belief"), toggle('actual', 'Actual path'),
      toggle('ghosts', 'Future poses'), toggle('wrench', 'Applied wrenches'), toggle('bearing', 'Bearing forces'), toggle('ellipsoids', 'Ellipsoids at the tool'),
      toggle('axes', 'Screw axes'), toggle('frames', 'Link frames')),
    field('Trail length', numberInput({ value: app.view.trail, min: 0.2, max: 40, onChange: (v) => { app.view.trail = v; app.save(); } }), 'seconds of actual path'),
    field('Chart window', numberInput({ value: app.view.plotWindow, min: 1, max: 40, onChange: (v) => { app.view.plotWindow = v; app.save(); } }), 'seconds'),
    button('Reset camera', () => app.scene.resetView(), 'small'),
  ));

  root.append(section('Save and load this setup', false, setupControls(app)));

  // ----- live readout -----
  const live = el('dl', { class: 'kv' });
  root.prepend(section('Now', true, live));
  return {
    el: root,
    refresh() {
      const s = app.snap;
      if (!s) return;
      const a = (v) => fmtVec(v.map((x) => app.toUI(x)), app.view.units === 'deg' ? 1 : 3);
      live.replaceChildren(
        el('dt', {}, `θ (${app.angleUnit})`), el('dd', {}, a(s.theta)),
        el('dt', {}, `error (${app.angleUnit})`), el('dd', {}, a(s.e)),
        el('dt', {}, 'τ (N·m)'), el('dd', {}, fmtVec(s.tau, 2)),
        el('dt', {}, 'energy (J)'), el('dd', {}, `${fmt(s.energy.E, 3)}  (balance ${fmt(s.energyBalance, 6)})`),
      );
    },
  };
}
