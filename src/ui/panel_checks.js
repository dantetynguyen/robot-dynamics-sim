// panel_checks.js — the Checks tab: physical consistency of the equations
// that are running right now (see src/sim/checks.js).

import { runChecks } from '../sim/checks.js';
import { el, section } from './dom.js';
import { fmt } from './format.js';

export function checksPanel(app) {
  const plant = el('div'), model = el('div'), edits = el('div');
  const table = (rows) => el('table', { class: 'data' },
    el('tr', {}, el('th', {}, 'Check'), el('th', {}, 'Measured'), el('th', {}, 'Limit'), el('th', {}, '')),
    rows.map((c) => el('tr', { title: c.why },
      el('td', {}, c.label, el('div', { class: 'cite', style: { opacity: 0.9 } }, c.why)),
      el('td', { class: 'n' }, fmt(c.value, 3)), el('td', { class: 'n' }, `${c.dir === '>=' ? '≥' : '≤'} ${fmt(c.limit, 3)}`),
      el('td', {}, el('span', { class: `chip ${c.pass ? 'ok' : 'bad'}` }, c.pass ? '✓ holds' : '✗ broken')))));
  const root = el('div', {},
    el('p', { class: 'note' }, 'Each row is a property every correct rigid-body model has. Break an equation in the Equations tab and see which rows fail: the pattern tells you what that equation was responsible for.'),
    edits,
    section('True plant', true, plant),
    section("Controller's model", true, model));
  const refresh = () => {
    const sim = app.sim;
    const list = [['plant', 'true plant'], ['model', "controller's model"], ['shared', 'shared']].flatMap(([k, name]) =>
      sim.equations[k].editedIds().map((id) => `${sim.equations[k].entries.get(id).slot.title} (${name})`));
    edits.replaceChildren(el('p', { class: `status-line ${list.length ? 'warn' : 'ok'}` },
      list.length ? `Edited equations: ${list.join('; ')}.` : 'All equations are the reference versions.'));
    plant.replaceChildren(table(runChecks(sim, 'plant')));
    model.replaceChildren(table(runChecks(sim, 'model')));
  };
  refresh();
  let last = 0;
  return { el: root, refresh() { const now = performance.now(); if (now - last > 900) { last = now; refresh(); } } };
}
