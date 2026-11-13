// panel_learn.js — the Learn tab: guided experiments, and the main equations
// shown with the numbers they have at this instant.

import { Simulation } from '../sim/simulation.js';
import { asymmetry, eigSym } from '../math/linalg.js';
import { EXPERIMENTS } from '../teach/experiments.js';
import { button, el, section } from './dom.js';
import { fmt, fmtVec, tex, texMatrix } from './format.js';

function runExperiment(app, ex) {
  // start from clean defaults so experiments never inherit leftovers
  const fresh = new Simulation(ex.config ?? {});
  const view = {
    mode: ex.mode, tab: 'learn', experiment: ex.id, torqueJoint: 0, chart4: 'energy', ...(ex.view ?? {}),
    show: { desired: true, predicted: true, belief: true, actual: true, ghosts: true, frames: false, axes: false, ellipsoids: false, bearing: false, wrench: true, jacobian: true, poe: false, ...(ex.view?.show ?? {}) },
  };
  if (ex.view?.kin) view.kin = { theta: ex.view.kin.theta.slice(), dtheta: ex.view.kin.dtheta.slice() };
  app.setRunning(false);
  app.loadSetup(fresh.toJSON(), view);
  if (ex.after) ex.after(app);
  app.view.tab = 'learn';
  app.setMode(ex.mode, 'learn');
  app.buildTopbar();
}

export function learnPanel(app) {
  const mode = app.view.mode;
  const root = el('div');
  const list = EXPERIMENTS.filter((e) => e.mode === mode), other = EXPERIMENTS.filter((e) => e.mode !== mode);
  const card = (ex) => {
    const active = app.view.experiment === ex.id;
    return el('div', { class: `card${active ? ' active' : ''}` },
      el('h4', {}, ex.title), el('div', { class: 'concept' }, ex.concept),
      active ? [
        el('p', {}, ex.todo),
        ex.actions ? el('div', { class: 'row' }, ex.actions.map((a) => button(a.label, () => a.run(app), 'small'))) : null,
        el('p', { class: 'watch' }, el('b', {}, 'Watch. '), ex.watch),
        el('p', {}, el('b', {}, 'Why. '), ex.why),
        el('div', { class: 'cite' }, ex.cite),
        el('div', { class: 'row' },
          ex.mode === 'dynamics' ? button('Run', () => app.setRunning(true), 'small primary') : null,
          button('Set up again', () => runExperiment(app, ex), 'small'),
          ex.view?.tab ? button(`Open the ${ex.view.tab === 'wrench' ? 'Wrenches' : ex.view.tab[0].toUpperCase() + ex.view.tab.slice(1)} tab`, () => { app.view.tab = ex.view.tab; app.setMode(app.view.mode, ex.view.tab); }, 'small') : null),
      ] : el('div', { class: 'row' }, button('Set up this experiment', () => runExperiment(app, ex), 'small')));
  };
  root.append(
    el('p', { class: 'note' }, 'Each experiment loads a ready-made setup, tells you what to do and what to watch, and says why it happens. Setting one up replaces the current setup.'),
    ...list.map(card),
    section(mode === 'dynamics' ? 'Kinematics experiments' : 'Dynamics experiments', false, other.map(card)));

  if (mode === 'dynamics') {
    const eom = el('div'), mass = el('div'), ctrl = el('div');
    root.append(
      section('The equation of motion, right now', true, tex('\\tau = M(\\theta)\\,\\ddot\\theta + c(\\theta,\\dot\\theta) + g(\\theta) + \\textstyle\\sum_i D_i J_s^{\\mathsf T}F^{ext}_{0,i} + \\tau_{fric}', true), eom,
        el('p', { class: 'note' }, 'M θ̈ is the torque that goes into accelerating the links; c is what their velocities demand (Coriolis and centripetal); g holds the arm up; the last terms balance outside wrenches and friction. The torque chart plots these same pieces for one joint.'),
        el('p', { class: 'dh' }, 'This is D(q)q̈ + C(q,q̇)q̇ + g(q) = τ from DH-based dynamics. M is D, and c is C(q,q̇)q̇ already multiplied out.'),
        el('div', { class: 'cite' }, 'Topic 13 p. 3 (D_i and per-link wrenches are in the notes only); MR Eq. 8.10 and §8.4')),
      section('Mass matrix M(θ)', true, mass,
        el('p', { class: 'note' }, 'M is the inertia each joint feels in this pose. It is built by calling inverse dynamics three times with θ̇ = 0, g = 0 and θ̈ = e₁, e₂, e₃. It must be symmetric and positive definite because ½ θ̇ᵀMθ̇ is kinetic energy.'),
        el('div', { class: 'cite' }, 'Topic 13 pp. 4–5; MR §8.5 and Eq. 8.57')),
      section('Recursive Newton–Euler', false,
        tex('\\mathcal V_i=\\mathcal A_i\\dot\\theta_i+\\mathrm{Ad}_{T_{i,i-1}}\\mathcal V_{i-1}', true),
        tex('\\dot{\\mathcal V}_i=\\mathcal A_i\\ddot\\theta_i+\\mathrm{ad}_{\\mathcal V_i}\\mathcal A_i\\dot\\theta_i+\\mathrm{Ad}_{T_{i,i-1}}\\dot{\\mathcal V}_{i-1}', true),
        tex('F_i=G_i\\dot{\\mathcal V}_i-\\mathrm{ad}_{\\mathcal V_i}^{\\mathsf T}G_i\\mathcal V_i+\\mathrm{Ad}_{T_{i+1,i}}^{\\mathsf T}F_{i+1}+\\mathrm{Ad}_{T_{0,i}}^{\\mathsf T}F^{ext}_{0,i},\\qquad \\tau_i=F_i^{\\mathsf T}\\mathcal A_i', true),
        el('p', { class: 'note' }, 'Motion flows out from the base (a joint moves everything after it); loads flow back from the tool (a joint carries everything after it). Gravity enters as a base acceleration V̇₀ = (0, −g). All five lines are editable in the Equations tab.'),
        el('p', { class: 'dh' }, 'Same outward/inward recursion as classical Newton–Euler with DH frames. Here one 6×6 adjoint does the rotation and the lever arm at once, and ad_V collects every ω× term.'),
        el('div', { class: 'cite' }, 'Topic 11 pp. 10–14; MR Eqs. 8.50–8.54')),
      section('Control law in use', true, ctrl,
        el('div', { class: 'cite' }, 'MR Ch. 11 (the course notes do not cover control)')),
    );
    const refresh = () => {
      const s = app.snap;
      if (!s) return;
      const sum = s.parts.inertial.map((v, i) => v + s.parts.c[i] + s.parts.g[i] + s.parts.ext[i] + s.parts.friction[i]);
      eom.replaceChildren(el('table', { class: 'data' },
        el('tr', {}, el('th', {}, 'N·m'), ...s.tau.map((_, i) => el('th', {}, `joint ${i + 1}`))),
        ...[['M θ̈', s.parts.inertial], ['c', s.parts.c], ['g', s.parts.g], ['wrenches', s.parts.ext], ['friction', s.parts.friction], ['sum', sum], ['τ commanded', s.tau]]
          .map(([name, v]) => el('tr', {}, el('td', {}, name), ...v.map((x) => el('td', { class: 'n' }, fmt(x, 3)))))));
      const sym = s.Mq.map((r, i) => r.map((v, j) => (v + s.Mq[j][i]) / 2));
      mass.replaceChildren(tex(`M(\\theta)=${texMatrix(s.Mq, 4)}\\ \\mathrm{kg\\,m^2}`, true),
        el('dl', { class: 'kv' },
          el('dt', {}, 'largest |M − Mᵀ| entry'), el('dd', {}, fmt(asymmetry(s.Mq), 3)),
          el('dt', {}, 'eigenvalues'), el('dd', {}, fmtVec(eigSym(sym).values, 4))));
      const g = app.sim.config.gains, slot = app.sim.controller.slot;
      ctrl.replaceChildren(
        el('p', {}, el('b', {}, app.sim.controller.name), ': ', el('code', {}, `τ = ${app.sim.equations.shared.text(slot)}`)),
        el('dl', { class: 'kv' }, el('dt', {}, 'Kp'), el('dd', {}, fmtVec(g.Kp, 2)), el('dt', {}, 'Kd'), el('dd', {}, fmtVec(g.Kd, 2)), el('dt', {}, 'Ki'), el('dd', {}, fmtVec(g.Ki, 2))),
        el('p', { class: 'note' }, app.sim.equations.shared.entries.get(slot).slot.note));
    };
    return { el: root, refresh };
  }
  return { el: root };
}
