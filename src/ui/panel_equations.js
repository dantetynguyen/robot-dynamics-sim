// panel_equations.js — the Equations tab: edit the equations the simulator
// runs, as text, while it runs.

import { SLOTS } from '../equations/registry.js';
import { button, el, select } from './dom.js';

const GROUP_NOTES = {
  'Recursive Newton–Euler': 'The inverse-dynamics recursion. M(θ), c and g are all built from these five lines, so an edit here changes the whole model.',
  'Rigid body': 'How one link resists motion.',
  Controller: 'The control laws. The one selected in the Control tab is the one that runs.',
  Trajectory: 'Time scalings for point-to-point moves; derivatives of an edited s(t) are taken numerically.',
  Wrenches: 'Used by the Wrenches tab and by the Checks tab.',
};

export function equationsPanel(app) {
  const sim = app.sim;
  // Where an edit to a model equation goes.
  let target = app.view.eqTarget ?? 'plant';
  const root = el('div');
  const targetSel = select([['plant', 'True plant'], ['model', "Controller's model"], ['both', 'Both']], target, (v) => { target = v; app.view.eqTarget = v; app.renderTab(); });
  root.append(
    el('p', { class: 'note' }, 'Type over any equation. It is checked as you type: green = same numbers as the reference, amber = a valid but different equation (now running), red = cannot be used (the last working version keeps running).'),
    el('p', { class: 'note' }, 'Syntax is MATLAB-like: ', el('code', {}, "A*B  A'  A.*B  V(1:3)  [a, b; c, d]  ad(V)  Ad(T)  skew(w)  blkdiag(A,B)  eye(3)  zeros(3,1)"), '.'),
    el('div', { class: 'row' }, el('span', { class: 'field-label' }, 'Model edits apply to'), targetSel),
    el('p', { class: 'note' }, 'Editing only the true plant shows what wrong physics does. Editing only the controller\'s model shows what a wrong model does to control.'),
  );

  const setsFor = (slot) => (slot.scope === 'shared' ? [sim.equations.shared]
    : slot.scope === 'plant' ? [sim.equations.plant]
      : target === 'both' ? [sim.equations.plant, sim.equations.model] : [sim.equations[target]]);

  let group = null;
  for (const slot of SLOTS) {
    if (slot.group !== group) {
      group = slot.group;
      root.append(el('h3', {}, group), el('p', { class: 'note' }, GROUP_NOTES[group] ?? ''));
    }
    const sets = setsFor(slot);
    const primary = sets[0];
    const area = el('textarea', { rows: Math.max(1, Math.ceil(primary.text(slot.id).length / 46)), spellcheck: 'false', autocapitalize: 'off', autocomplete: 'off', 'aria-label': slot.title });
    area.value = primary.entries.get(slot.id).pendingText ?? primary.text(slot.id);
    const status = el('div', { class: 'status-line' });
    const paint = () => {
      const st = primary.status(slot.id);
      area.className = `state-${st.state}`;
      status.className = `status-line ${st.state === 'error' ? 'bad' : st.state === 'differs' ? 'warn' : st.state === 'match' ? 'ok' : ''}`;
      status.textContent = st.state === 'reference' ? 'Reference equation.' : st.state === 'match' ? `✓ Edited; ${st.message}.` : st.state === 'differs' ? `≠ Running your version: ${st.message}.` : `✗ ${st.message}.`;
    };
    const commit = (text) => {
      for (const set of sets) {
        const entry = set.entries.get(slot.id);
        entry.pendingText = undefined;
        set.set(slot.id, text);
      }
      app.changed();
      paint();
    };
    area.addEventListener('input', () => commit(area.value));
    const where = slot.scope === 'model' ? (target === 'both' ? 'plant + model' : target === 'plant' ? 'true plant' : "controller's model") : slot.scope === 'plant' ? 'true plant only' : null;
    root.append(el('div', { class: 'eq' },
      el('div', { class: 'eq-head' }, el('span', { class: 'eq-title' }, slot.title), where ? el('span', { class: 'chip' }, where) : null),
      el('div', { class: 'eq-edit' }, el('span', { class: 'eq-lhs' }, slot.lhs), area),
      status,
      el('div', { class: 'row' },
        button('Restore reference', () => { area.value = slot.seed; commit(slot.seed); }, 'small'),
        (slot.wrong ?? []).map((w) => button(`Try: ${w.label}`, () => { area.value = w.text; commit(w.text); }, 'small'))),
      el('p', { class: 'note' }, slot.note),
      el('details', {}, el('summary', {}, 'Variables you can use'),
        el('dl', { class: 'kv' }, Object.entries(slot.vars).flatMap(([k, d]) => [el('dt', {}, el('code', {}, k)), el('dd', {}, d)]))),
      el('div', { class: 'cite' }, slot.cite)));
    paint();
  }
  return { el: root };
}
