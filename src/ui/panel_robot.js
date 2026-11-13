// panel_robot.js — the Robot tab: every parameter of the arm.
// Edits go into a working copy and are applied only if the whole description
// is valid, so a half-typed number never reaches the simulation.

import { cloneSpec, specProblems } from '../robot/model.js';
import { DEFAULT_PRESET, PRESETS } from '../robot/presets.js';
import { button, el, field, numberInput, section, vectorInput } from './dom.js';
import { fmtVec, matrixTable } from './format.js';

export function robotPanel(app) {
  const sim = app.sim;
  const draft = cloneSpec(sim.config.spec);
  const problemsEl = el('div');
  const apply = () => {
    const problems = specProblems(draft);
    problemsEl.replaceChildren(problems.length ? el('div', { class: 'problems' }, 'Not applied yet:', el('ul', {}, problems.map((p) => el('li', {}, p)))) : '');
    if (problems.length) return;
    sim.config.spec = cloneSpec(draft);
    app.changed({ robot: true });
  };
  const vec = (values, onChange, opts = {}) => vectorInput({ values, onChange: (v) => { onChange(v); apply(); }, ...opts });
  const num = (value, onChange, opts = {}) => numberInput({ value, onChange: (v) => { onChange(v); apply(); }, ...opts });
  const root = el('div', {},
    el('p', { class: 'note' }, 'SI units. The space frame {0} sits at the base with z up. Each link frame is at its centre of mass. The home configuration is the pose at θ = 0.'),
    problemsEl,
    section('World', true,
      field('Gravity vector', vec(draft.gravity, (v) => { draft.gravity = v; }, { min: -100, max: 100 }), 'm/s² in {0}. Try [0, 0, 0] for free space or [0, 0, −1.62] for the Moon.')));

  draft.joints.forEach((j, i) => {
    const link = draft.links[i], shape = link.shape;
    const pos = [link.Mlink[0][3], link.Mlink[1][3], link.Mlink[2][3]];
    root.append(section(`Joint ${i + 1} and link ${i + 1}`, i === 0,
      j.S ? field('Screw axis S', el('span', { class: 'vec6' }, vec(j.S, (v) => { j.S = v; })), '(ω, v) in {0} at home') : [
        field('Axis direction ŝ', vec(j.sHat, (v) => { j.sHat = v; }, { min: -1, max: 1 }), 'in {0} at home; it is normalised for you'),
        field('Point q on the axis', vec(j.q, (v) => { j.q = v; }, { min: -10, max: 10 }), 'm. S = (ŝ, −ŝ × q)'),
      ],
      field('Joint damping b', num(draft.damping[i], (v) => { draft.damping[i] = v; }, { min: 0, max: 1000 }), 'N·m·s/rad, viscous. 0 = frictionless.'),
      field('Link mass', num(link.mass, (v) => { link.mass = v; }, { min: 1e-4, max: 1e4 }), 'kg'),
      field('Centre of mass', vec(pos, (v) => { for (let k = 0; k < 3; k++) link.Mlink[k][3] = v[k]; }, { min: -10, max: 10 }), 'm in {0} at home (position part of M_i)'),
      shape.type === 'cylinder' && [
        field('Cylinder radius', num(shape.r, (v) => { shape.r = v; }, { min: 1e-4, max: 5 }), 'm'),
        field('Cylinder length', num(shape.h, (v) => { shape.h = v; }, { min: 1e-4, max: 10 }), 'm, along the link frame\'s z axis'),
      ],
      shape.type === 'box' && field('Box size', vec([shape.lx, shape.ly, shape.lz], (v) => { [shape.lx, shape.ly, shape.lz] = v; }, { min: 1e-4, max: 10 }), 'm along local x, y, z'),
      shape.type === 'ellipsoid' && field('Semi-axes', vec([shape.a, shape.b, shape.c], (v) => { [shape.a, shape.b, shape.c] = v; }, { min: 1e-4, max: 10 }), 'm'),
      shape.type === 'matrix' && el('p', { class: 'note' }, 'This link uses a raw 3×3 inertia matrix; edit it in a saved setup file.'),
    ));
  });

  root.append(section('Tool frame', false,
    field('Tool point', vec([draft.Mee[0][3], draft.Mee[1][3], draft.Mee[2][3]], (v) => { for (let k = 0; k < 3; k++) draft.Mee[k][3] = v[k]; }, { min: -10, max: 10 }), 'm in {0} at home (position part of M)')));

  const derived = el('div');
  root.append(section('Derived quantities', false, derived),
    el('div', { class: 'row' }, button('Restore the default arm', () => { sim.config.spec = PRESETS[DEFAULT_PRESET](); app.changed({ robot: true, reset: true }); app.renderTab(); }, 'small')));

  const showDerived = () => {
    const r = app.sim.robot;
    derived.replaceChildren(...r.S.flatMap((S, i) => [
      el('h3', {}, `Link ${i + 1}`),
      el('dl', { class: 'kv' },
        el('dt', {}, `S${i + 1} in {0}`), el('dd', {}, fmtVec(S)),
        el('dt', {}, `A${i + 1} in {${i + 1}}`), el('dd', {}, fmtVec(r.A[i]))),
      el('div', { class: 'matrow' }, `G${i + 1} =`, matrixTable(r.G[i], 4)),
    ]));
  };
  showDerived();
  let seen = sim.generation;
  return { el: root, refresh() { if (app.sim.generation !== seen) { seen = app.sim.generation; showDerived(); } } };
}
