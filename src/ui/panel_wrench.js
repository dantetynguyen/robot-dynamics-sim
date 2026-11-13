// panel_wrench.js — the Wrenches tab: apply wrenches to any link in a frame
// of your choice and see them as screws, in three frames, as joint torques,
// as reciprocal (zero-torque) directions, and as motor/bearing loads.

import { externalTorques } from '../dynamics/terms.js';
import { matVec } from '../math/linalg.js';
import { adjoint, transInv } from '../math/lie.js';
import { pointEllipsoids, power, reciprocalBasis, wrenchToScrew } from '../wrench/wrench.js';
import { button, checkbox, el, section, select, vectorInput } from './dom.js';
import { fmt, fmtVec, tex } from './format.js';

const LABELS = ['τx', 'τy', 'τz', 'fx', 'fy', 'fz'];

export function wrenchPanel(app) {
  const sim = app.sim, cfg = sim.config, N = sim.N;
  const root = el('div');
  const rerender = () => { app.changed(); app.renderTab(); };
  const targets = [...Array.from({ length: N }, (_, i) => [i, `Link ${i + 1}`]), [N, 'Tool frame']];
  const frames = [['aligned', 'At the target, axes parallel to {0}'], ['body', "The target's own frame"], ['space', 'Space frame {0}']];
  const add = (w) => { cfg.wrenches.push({ enabled: true, link: N, frame: 'aligned', W: [0, 0, 0, 0, 0, 0], ...w }); rerender(); };

  root.append(
    el('p', { class: 'note' }, 'A wrench is F = (τ, f): moment on top, force below. Type the six numbers in the frame you pick; the simulator moves them to {0} with ',
      tex('F_b = \\mathrm{Ad}_{T_{ab}}^{\\mathsf T} F_a'), '.'),
    el('div', { class: 'row' }, el('span', { class: 'field-label' }, 'The numbers mean'),
      select([['applied', 'the wrench applied TO the link (a push)'], ['notes', "F^ext of the notes: wrench the link exerts on the world"]], cfg.wrenchSense, (v) => { cfg.wrenchSense = v; rerender(); })),
    el('p', { class: 'note' }, 'The two senses are opposite: F^ext = −(applied). The equations of motion use F^ext, ',
      tex('\\tau = M\\ddot\\theta + h + \\textstyle\\sum_i D_i J_s^{\\mathsf T} F^{ext}_{0,i}'), '.'),
  );

  cfg.wrenches.forEach((w, k) => {
    root.append(el('div', { class: 'card' },
      el('div', { class: 'row' },
        checkbox(`Wrench ${k + 1}`, w.enabled, (v) => { w.enabled = v; app.changed(); }),
        select(targets, w.link, (v) => { w.link = Number(v); app.changed(); }),
        button('Remove', () => { cfg.wrenches.splice(k, 1); rerender(); }, 'small quiet')),
      el('div', { class: 'row' }, el('span', { class: 'field-label' }, 'Expressed in'), select(frames, w.frame, (v) => { w.frame = v; app.changed(); })),
      el('div', { class: 'row vec6' }, vectorInput({ values: w.W, labels: LABELS, min: -1e4, max: 1e4, onChange: (v) => { w.W = v; app.changed(); } })),
      el('div', { class: 'cite', style: { opacity: 0.9 } }, '[τx, τy, τz, fx, fy, fz] in N·m and N')));
  });
  const recip = el('div', { class: 'row' });
  root.append(
    el('div', { class: 'row' },
      button('Add: 15 N down at the tool', () => add({ W: [0, 0, 0, 0, 0, -15] }), 'small'),
      button('Add: 2 N·m twist at the tool', () => add({ frame: 'body', W: [2, 0, 0, 0, 0, 0] }), 'small'),
      button('Add an empty wrench', () => add({}), 'small')),
    recip);
  if (cfg.wrenches.length) root.append(el('div', { class: 'row' }, button('Remove all wrenches', () => { cfg.wrenches.length = 0; rerender(); }, 'small quiet')));

  const screwEl = el('div'), framesEl = el('div'), nullEl = el('div'), jointEl = el('div'), ellEl = el('div');
  root.append(
    section('First active wrench as a screw', true, screwEl,
      el('p', { class: 'note' }, 'Poinsot: any wrench is a force along some line plus a couple about that same line. For a wrench, pitch h = couple ÷ force: h = 0 is a pure force, h = ∞ a pure couple (the reverse of twists).'),
      el('div', { class: 'cite' }, 'Topic 10 pp. 1–3 (the wrench screw form is in the notes only)')),
    section('The same wrench in three frames', true, framesEl,
      el('p', { class: 'note' }, 'The six numbers change with the frame. The power VᵀF it exchanges with the moving link, and the joint torques it costs, do not.'),
      el('div', { class: 'cite' }, 'Topic 10 pp. 3, 7; MR Eqs. 3.94–3.95')),
    section('Wrenches the arm cannot feel: null(J_sᵀ)', true, nullEl,
      el('p', { class: 'note' }, 'Any tool wrench in this space is reciprocal to every joint screw, so it costs zero motor torque: the structure carries it. A 3-joint arm leaves 6 − 3 = 3 such directions (more at a singularity). They change with the pose.'),
      el('div', { class: 'cite' }, 'Topic 10 p. 5; MR §5.2')),
    section('Joint wrenches: motor part and bearing part', true, jointEl,
      el('p', { class: 'note' }, 'F_i from the RNE backward pass is everything passing through joint i. The motor supplies only its component along the joint screw, ', tex('\\tau_i = F_i^{\\mathsf T}\\mathcal A_i'), '; the bearing carries the other five dimensions.'),
      el('div', { class: 'cite' }, 'Topic 10 p. 6; Topic 11 p. 13; MR Eq. 8.54')),
    section('Ellipsoids at the tool point', false, ellEl,
      el('p', { class: 'note' }, 'Same axes, reciprocal lengths: where the tool moves easily it pushes weakly. Turn them on under Control → Display.'),
      el('div', { class: 'cite' }, 'Topic 10 pp. 8–9 (as corrected in class to (JJᵀ)⁻¹); MR §5.4')),
  );

  const refresh = () => {
    const s = app.snap;
    if (!s) return;
    const w = cfg.wrenches.find((x) => x.enabled);
    // reciprocal basis buttons
    const basis = reciprocalBasis(s.Js).basis;
    recip.replaceChildren(...basis.map((b, k) => button(`Add: reciprocal wrench ${k + 1} (40 N scale)`, () => {
      const scale = 40 / Math.max(Math.hypot(b[3], b[4], b[5]), 0.2);
      add({ frame: 'space', link: N, W: b.map((x) => Number((x * scale).toFixed(6))) });
    }, 'small')));
    nullEl.replaceChildren(el('table', { class: 'data' },
      el('tr', {}, el('th', {}, '#'), el('th', {}, 'F in {0}  (τ, f)'), el('th', {}, 'J_sᵀF (N·m)')),
      basis.map((b, k) => el('tr', {}, el('td', {}, k + 1), el('td', {}, fmtVec(b, 3)), el('td', {}, fmtVec(externalTorques(sim.robot, s.theta, [...new Array(N).fill(null), b]), 6))))));

    if (!w) {
      screwEl.replaceChildren(el('p', { class: 'note' }, 'Add a wrench above to see it here.'));
      framesEl.replaceChildren();
    } else {
      const F0 = sim.wrenchInSpace(w, s.T0);
      const sc = wrenchToScrew(F0);
      screwEl.replaceChildren(el('dl', { class: 'kv' },
        el('dt', {}, 'kind'), el('dd', {}, sc.kind === 'force' ? (Math.abs(sc.h) < 1e-9 ? 'pure force (h = 0)' : 'force + couple about its line') : sc.kind === 'couple' ? 'pure couple (h = ∞)' : 'zero'),
        el('dt', {}, 'axis ŝ'), el('dd', {}, fmtVec(sc.sHat)),
        el('dt', {}, 'point q on the axis'), el('dd', {}, `${fmtVec(sc.q)} m`),
        el('dt', {}, 'pitch h'), el('dd', {}, Number.isFinite(sc.h) ? `${fmt(sc.h, 4)} m` : '∞'),
        el('dt', {}, 'magnitude'), el('dd', {}, sc.kind === 'couple' ? `${fmt(sc.magnitude, 3)} N·m` : `${fmt(sc.magnitude, 3)} N`)));

      // twist of the target link: V_0 = J_s D_L θ̇ (joints after the link do not move it)
      const reach = w.link === N ? N : w.link + 1;
      const V0 = matVec(s.Js, s.dtheta.map((d, i) => (i < reach ? d : 0)));
      const transform = sim.equations.shared.fn('wrench.transform');
      const rows = [['space {0}', 'space'], ["target's own frame", 'body'], ['at target, axes ∥ {0}', 'aligned']].map(([name, frame]) => {
        const T0f = sim.wrenchFrame({ ...w, frame }, s.T0);
        const Ff = frame === 'space' ? F0 : transform({ Tab: T0f, Fa: F0 });      // F_f = Ad_{T_0f}ᵀ F_0
        const Vf = frame === 'space' ? V0 : matVec(adjoint(transInv(T0f)), V0);   // V_f = Ad_{T_f0} V_0
        return el('tr', {}, el('td', {}, name), el('td', {}, fmtVec(Ff, 3)), el('td', { class: 'n' }, fmt(power(Vf, Ff), 6)));
      });
      const notes = cfg.wrenchSense === 'applied' ? F0.map((x) => -x) : F0;
      const Fext = new Array(N + 1).fill(null);
      Fext[w.link] = notes;
      framesEl.replaceChildren(
        el('table', { class: 'data' }, el('tr', {}, el('th', {}, 'Frame'), el('th', {}, '(τ, f) as typed sense'), el('th', {}, 'VᵀF (W)')), rows),
        el('dl', { class: 'kv' }, el('dt', {}, 'Torque it costs, D_i J_sᵀ F^ext'), el('dd', {}, `${fmtVec(externalTorques(sim.robot, s.theta, Fext), 4)} N·m`)));
    }

    jointEl.replaceChildren(el('table', { class: 'data' },
      el('tr', {}, el('th', {}, 'Joint'), el('th', {}, 'motor τ_i (N·m)'), el('th', {}, 'bearing: thrust (N)'), el('th', {}, 'radial (N)'), el('th', {}, 'bending (N·m)')),
      s.joints.map((j, i) => el('tr', {}, el('td', {}, i + 1), el('td', { class: 'n' }, fmt(j.tauMotor, 3)), el('td', { class: 'n' }, fmt(j.axialForce, 2)),
        el('td', { class: 'n' }, fmt(j.radialForce, 2)), el('td', { class: 'n' }, fmt(j.bendingMoment, 3))))));

    const e = pointEllipsoids(s.Jp);
    ellEl.replaceChildren(el('dl', { class: 'kv' },
      el('dt', {}, 'velocity semi-axes √λ'), el('dd', {}, `${fmtVec(e.manipulability, 3)} m/s per rad/s`),
      el('dt', {}, 'force semi-axes 1/√λ'), el('dd', {}, `${fmtVec(e.force, 2)} N per N·m`),
      el('dt', {}, 'μ₁ (longest ÷ shortest)'), el('dd', {}, fmt(e.mu1, 2))));
  };
  return { el: root, refresh };
}
