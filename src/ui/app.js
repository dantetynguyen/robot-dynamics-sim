// app.js — the application shell: top bar, tabs, the frame loop, and the
// glue that turns simulation state into what the 3D view and charts show.
//
// Data flow each animation frame:
//   Simulation.step()  ->  Simulation.snapshot()  ->  updateScene() / charts / panels
// Panels change `sim.config` (or the equation sets) and call app.changed().

import { jacobianSpace, pointJacobian } from '../dynamics/kinematics.js';
import { matVec } from '../math/linalg.js';
import { screwGeometry } from '../math/lie.js';
import { buildRobot } from '../robot/model.js';
import { Simulation } from '../sim/simulation.js';
import { pointEllipsoids, wrenchToScrew } from '../wrench/wrench.js';
import { button, clear, el, select } from './dom.js';
import { DEG, fmt } from './format.js';
import { poseGeometry } from './geometry.js';
import { StripChart } from './plots.js';
import { PALETTE, Scene3D } from './scene.js';
import { controlPanel } from './panel_control.js';
import { robotPanel } from './panel_robot.js';
import { wrenchPanel } from './panel_wrench.js';
import { equationsPanel } from './panel_equations.js';
import { learnPanel } from './panel_learn.js';
import { checksPanel } from './panel_checks.js';
import { kinematicsPanel } from './panel_kinematics.js';

const STORAGE_KEY = 'robot-dynamics-sim/setup-v1';

const TABS = {
  dynamics: [['control', 'Control'], ['wrench', 'Wrenches'], ['equations', 'Equations'], ['learn', 'Learn'], ['checks', 'Checks'], ['robot', 'Robot']],
  kinematics: [['kinematics', 'Kinematics'], ['learn', 'Learn'], ['robot', 'Robot']],
};
const PANELS = {
  control: controlPanel, robot: robotPanel, wrench: wrenchPanel, equations: equationsPanel,
  learn: learnPanel, checks: checksPanel, kinematics: kinematicsPanel,
};

// How long arrows are drawn per unit of what they represent.
export const SCALE = { force: 0.01, moment: 0.05, bearing: 0.004, velocity: 0.25, angular: 0.1, manip: 0.25, forceEll: 0.02 };

export class App {
  constructor(root) {
    this.root = root;
    this.sim = new Simulation();
    this.view = {
      mode: 'dynamics', tab: 'control', units: 'deg', theme: 'dark', speed: 1, running: false,
      trail: 6, plotWindow: 8, torqueJoint: 0, chart4: 'energy',
      show: { desired: true, predicted: true, belief: true, actual: true, ghosts: true, frames: false, axes: false, ellipsoids: false, bearing: false, wrench: true, jacobian: true, poe: false },
      kin: { theta: [0.4, 0.6, -0.9], dtheta: [0.6, 0.4, -0.5] },
      experiment: null,
    };
    this.predDirty = true;
    this.lastPredAt = 0;
    this.predCost = 0;
    this.lastFrame = performance.now();
    this.budget = 0;
    this.rate = { sim: 0, wall: 0, value: 1 };
    this.timers = { charts: 0, panels: 0 };
    this.restore();
    document.documentElement.dataset.theme = this.view.theme;
    this.build();
    requestAnimationFrame((t) => this.frame(t));
  }

  // ---------- units ----------
  get angleUnit() { return this.view.units === 'deg' ? '°' : 'rad'; }
  toUI(rad) { return this.view.units === 'deg' ? rad / DEG : rad; }
  fromUI(v) { return this.view.units === 'deg' ? v * DEG : v; }

  // ---------- construction ----------
  build() {
    clear(this.root);
    this.viewport = el('div', { class: 'viewport' });
    this.chartsEl = el('div', { class: 'charts' });
    this.tabsEl = el('div', { class: 'tabs', role: 'tablist' });
    this.tabBody = el('div', { class: 'tab-body' });
    this.topbar = el('header', { class: 'topbar' });
    this.root.append(
      this.topbar,
      el('div', { class: 'main' },
        el('div', { class: 'stage' }, this.viewport, this.chartsEl),
        el('aside', { class: 'side' }, this.tabsEl, this.tabBody)),
    );
    this.scene = new Scene3D(this.viewport, this.view.theme);
    this.scene.setRobot(this.sim.robot);
    this.legend = el('div', { class: 'legend3d' });
    this.banner = el('div', { class: 'banner', hidden: true, role: 'alert' });
    this.viewport.append(this.legend, el('div', { class: 'viewhint' }, 'Drag to orbit, scroll to zoom, right-drag to pan'), this.banner);
    this.buildTopbar();
    this.buildCharts();
    this.setMode(this.view.mode, this.view.tab);
  }

  buildTopbar() {
    const v = this.view;
    const seg = (pairs, current, onPick) => el('div', { class: 'seg' }, pairs.map(([id, label]) =>
      el('button', { type: 'button', 'aria-pressed': String(id === current), onclick: () => onPick(id) }, label)));
    this.timeEl = el('span', { class: 'readout' });
    this.runBtn = button(v.running ? 'Pause' : 'Run', () => this.setRunning(!this.view.running), 'primary');
    clear(this.topbar).append(
      el('span', { class: 'brand' }, 'Robot Dynamics Simulator'),
      seg([['kinematics', 'Kinematics'], ['dynamics', 'Dynamics']], v.mode, (m) => this.setMode(m)),
      ...(v.mode === 'dynamics' ? [
        this.runBtn,
        button('Step', () => { this.setRunning(false); this.sim.step(Math.round(0.01 / this.sim.config.dt)); this.predDirty = true; }),
        button('Reset', () => this.reset()),
        el('label', { class: 'readout' }, 'Speed ', select([[0.1, '0.1×'], [0.25, '0.25×'], [0.5, '0.5×'], [1, '1×'], [2, '2×']], v.speed, (s) => { v.speed = Number(s); this.save(); })),
        this.timeEl,
      ] : []),
      el('span', { class: 'spacer' }),
      seg([['deg', 'deg'], ['rad', 'rad']], v.units, (u) => { v.units = u; this.save(); this.buildTopbar(); this.buildCharts(); this.renderTab(); }),
      button(v.theme === 'dark' ? 'Light theme' : 'Dark theme', () => this.setTheme(v.theme === 'dark' ? 'light' : 'dark'), 'quiet'),
    );
  }

  setTheme(theme) {
    this.view.theme = theme;
    document.documentElement.dataset.theme = theme;
    this.scene.setTheme(theme);
    this.buildTopbar();
    this.renderLegend();
    this.save();
  }

  setMode(mode, tab) {
    this.view.mode = mode;
    if (mode === 'kinematics') this.setRunning(false);
    this.chartsEl.hidden = mode !== 'dynamics';
    const tabs = TABS[mode];
    this.view.tab = tabs.some(([id]) => id === tab) ? tab : tabs[0][0];
    clear(this.tabsEl).append(...tabs.map(([id, label]) => el('button', {
      type: 'button', role: 'tab', 'aria-selected': String(id === this.view.tab),
      onclick: () => { this.view.tab = id; this.setMode(mode, id); },
    }, label)));
    this.buildTopbar(); // the top bar shows different controls in each mode
    this.renderTab();
    this.renderLegend();
    this.predDirty = true;
    this.save();
  }

  renderTab() {
    this.panel = PANELS[this.view.tab](this);
    clear(this.tabBody).append(this.panel.el);
    this.tabBody.scrollTop = 0;
    this.timers.panels = 0;
  }

  renderLegend() {
    const c = PALETTE[this.view.theme], hex = (n) => `#${n.toString(16).padStart(6, '0')}`;
    const item = (color, label, cls = '', note = '') => el('span', { class: 'legend-item' },
      el('i', { class: `swatch ${cls}`, style: { '--c': hex(color) } }), label, note ? el('small', {}, note) : null);
    const s = this.view.show;
    clear(this.legend).append(...(this.view.mode === 'dynamics' ? [
      s.desired && item(c.desired, 'Desired path', 'dashed', 'reference trajectory'),
      s.predicted && item(c.predicted, 'Predicted path', '', 'true plant, from now'),
      s.belief && item(c.belief, "Controller's belief", 'dashed thin', 'its own model, from now'),
      s.actual && item(c.actual, 'Actual path', '', 'what happened'),
      s.wrench && this.sim.config.wrenches.some((w) => w.enabled) && item(c.wrench, 'Applied wrench', '', 'force →, moment ↠'),
    ] : [
      item(c.joints[0], 'Joint 1'), item(c.joints[1], 'Joint 2'), item(c.joints[2], 'Joint 3'),
      s.jacobian && item(c.twist, 'Tool velocity', '', 'sum of the joint contributions'),
      s.ellipsoids && item(c.manip, 'Manipulability ellipsoid'), s.ellipsoids && item(c.force, 'Force ellipsoid'),
    ]).filter(Boolean));
  }

  buildCharts() {
    const u = this.angleUnit, j = ['--j1', '--j2', '--j3'], sub = ['₁', '₂', '₃'];
    const N = this.sim.N;
    const jointSel = select(Array.from({ length: N }, (_, i) => [i, `joint ${i + 1}`]), this.view.torqueJoint, (v) => { this.view.torqueJoint = Number(v); this.save(); });
    const c4Sel = select([['energy', 'Energy'], ['bearing', 'Bearing force']], this.view.chart4, (v) => { this.view.chart4 = v; this.configureChart4(); this.save(); });
    this.charts = {
      angles: new StripChart({
        title: 'Joint angles', unit: u,
        series: [...j.slice(0, N).map((c, i) => ({ key: `th${i}`, label: `θ${sub[i]}`, color: c })), ...j.slice(0, N).map((c, i) => ({ key: `thd${i}`, label: `θ${sub[i]} ref`, color: c, dashed: true }))],
      }),
      error: new StripChart({ title: 'Tracking error', unit: u, series: j.slice(0, N).map((c, i) => ({ key: `e${i}`, label: `e${sub[i]}`, color: c })) }),
      torque: new StripChart({
        title: 'Torque at', unit: 'N·m', controls: jointSel,
        series: [
          { key: 'tau', label: 'τ', color: '--ink' }, { key: 'inertial', label: 'M θ̈', color: '--s1' }, { key: 'c', label: 'c', color: '--s2' },
          { key: 'g', label: 'g', color: '--s3' }, { key: 'ext', label: 'wrench', color: '--s5' }, { key: 'friction', label: 'friction', color: '--s6' },
        ],
      }),
      fourth: new StripChart({ title: '', unit: 'J', controls: c4Sel, series: [] }),
    };
    this.configureChart4();
    clear(this.chartsEl).append(...Object.values(this.charts).map((c) => c.el));
  }

  configureChart4() {
    const sub = ['₁', '₂', '₃'];
    if (this.view.chart4 === 'energy') {
      this.charts.fourth.setSeries([
        { key: 'E', label: 'total E', color: '--ink' }, { key: 'K', label: 'kinetic', color: '--s2' },
        { key: 'P', label: 'potential', color: '--s1' }, { key: 'balance', label: 'E − E₀ − work', color: '--s8', dashed: true },
      ], '', 'J');
    } else {
      this.charts.fourth.setSeries(['--j1', '--j2', '--j3'].slice(0, this.sim.N).map((c, i) => ({ key: `b${i}`, label: `joint ${sub[i]}`, color: c })), '', 'N');
    }
  }

  // ---------- actions ----------
  setRunning(on) {
    this.view.running = on && !this.sim.fault;
    if (this.runBtn) this.runBtn.textContent = this.view.running ? 'Pause' : 'Run';
    this.lastFrame = performance.now();
    this.budget = 0;
  }

  reset() {
    this.sim.touch();
    this.sim.reset();
    this.predDirty = true;
    this.timers.charts = 0;
  }

  // Call after changing sim.config or an equation. `opts.reset` restarts the run.
  changed(opts = {}) {
    this.sim.touch();
    if (opts.robot) this.scene.setRobot(this.sim.robot);
    if (opts.reset) this.sim.reset();
    this.predDirty = true;
    this.renderLegend();
    this.save();
  }

  // Load a complete setup (from a file, the clipboard or an experiment).
  loadSetup(saved, view = {}) {
    const problems = this.sim.load(saved);
    if (problems.length) return problems;
    Object.assign(this.view, view, { show: { ...this.view.show, ...(view.show ?? {}) } });
    this.scene.setRobot(this.sim.robot);
    this.predDirty = true;
    this.buildTopbar();
    this.buildCharts();
    this.setMode(this.view.mode, this.view.tab);
    return [];
  }

  // ---------- persistence (this browser only) ----------
  save() {
    clearTimeout(this.saveTimer);
    this.saveTimer = setTimeout(() => {
      try {
        const { experiment, running, ...view } = this.view;
        localStorage.setItem(STORAGE_KEY, JSON.stringify({ setup: this.sim.toJSON(), view }));
      } catch (e) { /* storage unavailable: the app works without it */ }
    }, 400);
  }

  restore() {
    try {
      const stored = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? 'null');
      if (!stored) return;
      if (this.sim.load(stored.setup).length === 0 && stored.view) {
        Object.assign(this.view, stored.view, { show: { ...this.view.show, ...(stored.view.show ?? {}) }, running: false, experiment: null });
      }
    } catch (e) { /* ignore a corrupt or missing entry */ }
  }

  // ---------- frame loop ----------
  frame(now) {
    const wall = Math.min((now - this.lastFrame) / 1000, 0.1);
    this.lastFrame = now;
    const sim = this.sim;

    if (this.view.mode === 'dynamics' && this.view.running && !sim.fault) {
      this.budget += wall * this.view.speed;
      const dt = sim.config.dt;
      let steps = Math.floor(this.budget / dt);
      const t0 = performance.now(), before = sim.t;
      // Spend at most ~14 ms per frame on physics so the view stays responsive.
      while (steps > 0 && performance.now() - t0 < 14) {
        const n = Math.min(steps, 10);
        if (!sim.step(n)) break;
        steps -= n;
      }
      this.budget = steps > 0 ? 0 : this.budget - (sim.t - before); // fell behind: drop the backlog
      this.rate.sim += sim.t - before;
      this.rate.wall += wall * this.view.speed;
      if (this.rate.wall > 1) { this.rate.value = this.rate.sim / this.rate.wall; this.rate.sim = 0; this.rate.wall = 0; }
      if (sim.fault) this.setRunning(false);
    }

    try {
      this.updateScene(now);
    } catch (e) {
      this.showBanner('Could not draw this state', e.message);
    }
    this.scene.render();

    if (now - this.timers.charts > 66 && this.view.mode === 'dynamics') { this.timers.charts = now; this.updateCharts(); }
    if (now - this.timers.panels > 200) {
      this.timers.panels = now;
      try { this.panel?.refresh?.(); } catch (e) { /* a panel must never stop the loop */ }
      if (this.timeEl) {
        const behind = this.view.running && this.rate.value < 0.97;
        this.timeEl.replaceChildren('t = ', el('b', {}, `${fmt(sim.t, 2)} s`), behind ? ` (running at ${fmt(this.rate.value * 100, 0)}% of the set speed)` : '');
      }
    }
    requestAnimationFrame((t) => this.frame(t));
  }

  showBanner(title, text) {
    this.banner.hidden = !title;
    if (title) this.banner.replaceChildren(el('b', {}, title), text);
  }

  // ---------- 3D ----------
  updateScene(now) {
    const { sim, scene, view } = this;
    const colors = PALETTE[view.theme], show = view.show;
    const robot = sim.robot;
    const hide = (...names) => names.forEach((n) => scene.setPath(n, null, false));

    if (view.mode === 'kinematics') {
      this.showBanner(null);
      const theta = view.kin.theta, dtheta = view.kin.dtheta;
      const geom = poseGeometry(robot, theta);
      scene.updatePose(geom, { showFrames: show.frames });
      hide('desired', 'predicted', 'belief', 'actual');
      scene.setArrows('wrench', []); scene.setArrows('bearing', []);
      for (let i = 0; i < 4; i++) scene.setLine(`wrenchAxis${i}`, null);
      // PoE build-up: apply the exponentials one at a time, last joint first
      const ghosts = [];
      if (show.poe) {
        for (let k = robot.N; k >= 1; k--) ghosts.push(poseGeometry(robot, theta.map((t, i) => (i >= k ? t : 0))));
        const names = ['M (home)', 'e^[S₃]θ₃ M', 'e^[S₂]θ₂ e^[S₃]θ₃ M'];
        ghosts.forEach((g, i) => scene.setLabel(`poe${i}`, names[i] ?? '', g.tool));
      }
      for (let i = ghosts.length; i < 4; i++) scene.setLabel(`poe${i}`, null);
      scene.setGhosts(ghosts, 0.5);
      this.drawAxes(geom, show.axes || show.poe);
      // Joint-rate contributions to the tool velocity: ṗ = Σ_i J_p,i θ̇_i
      const Js = jacobianSpace(robot, theta), Jp = pointJacobian(Js, geom.tool);
      const arrows = [];
      if (show.jacobian) {
        let sum = [0, 0, 0];
        for (let i = 0; i < robot.N; i++) {
          const vec = [0, 1, 2].map((k) => Jp[k][i] * dtheta[i]);
          arrows.push({ from: geom.tool, vec: vec.map((x) => x * SCALE.velocity), color: colors.joints[i % 3], label: `J${i + 1}·θ̇${i + 1}` });
          sum = sum.map((x, k) => x + vec[k]);
        }
        arrows.push({ from: geom.tool, vec: sum.map((x) => x * SCALE.velocity), color: colors.twist, label: `ṗ = ${fmt(Math.hypot(...sum), 2)} m/s` });
        const Vs = matVec(Js, dtheta), w = Vs.slice(0, 3);
        if (Math.hypot(...w) > 1e-6) {
          const sg = screwGeometry(Vs);
          scene.setLine('twistAxis', [sg.q.map((x, k) => x - sg.sHat[k] * 0.6), sg.q.map((x, k) => x + sg.sHat[k] * 0.6)], { color: colors.twist, width: 1.4, dashed: true, opacity: 0.7 });
          scene.setLabel('twistAxis', 'screw axis of V_s', sg.q.map((x, k) => x + sg.sHat[k] * 0.6));
        } else { scene.setLine('twistAxis', null); scene.setLabel('twistAxis', null); }
      } else { scene.setLine('twistAxis', null); scene.setLabel('twistAxis', null); }
      scene.setArrows('jacobian', arrows);
      this.drawEllipsoids(Jp, geom.tool, show.ellipsoids);
      return;
    }

    // ----- dynamics mode -----
    scene.setArrows('jacobian', []); scene.setLine('twistAxis', null); scene.setLabel('twistAxis', null);
    for (let i = 0; i < 4; i++) scene.setLabel(`poe${i}`, null);
    const snap = sim.snapshot();
    this.snap = snap;
    this.showBanner(snap.fault ? `Simulation stopped at t = ${fmt(snap.fault.time ?? sim.t, 3)} s` : null,
      snap.fault ? `${snap.fault.message}. Fix the setting or equation, then press Reset.` : '');
    const geom = poseGeometry(robot, snap.theta);
    scene.updatePose(geom, { showFrames: show.frames });
    this.drawAxes(geom, show.axes);

    // actual trail
    const tMin = sim.t - view.trail;
    const trail = [];
    for (const r of sim.log) if (r.t >= tMin) trail.push(r.p);
    trail.push(snap.p);
    scene.setPath('actual', trail, show.actual);

    // predictions: recomputed when something changed, and a few times a
    // second while running; less often if they are expensive.
    const interval = Math.max(200, this.predCost * 5);
    if (this.predDirty || (view.running && now - this.lastPredAt > interval)) {
      const t0 = performance.now();
      this.desired = show.desired ? sim.desiredPath() : null;
      this.predicted = show.predicted || show.ghosts ? sim.predict({ plant: 'true' }) : null;
      this.belief = show.belief ? sim.predict({ plant: 'belief' }) : null;
      this.predCost = performance.now() - t0;
      this.lastPredAt = now;
      this.predDirty = false;
      scene.setPath('desired', this.desired, show.desired);
      scene.setPath('predicted', this.predicted?.points, show.predicted);
      scene.setPath('belief', this.belief?.points, show.belief);
      const ghosts = [];
      if (show.ghosts && this.predicted && this.predicted.thetas.length > 3) {
        const n = this.predicted.thetas.length - 1;
        for (const f of [1 / 3, 2 / 3, 1]) ghosts.push(poseGeometry(robot, this.predicted.thetas[Math.round(n * f)]));
      }
      scene.setGhosts(ghosts, 0.28);
    }

    // applied wrenches, drawn as screws: force along the line of action (→)
    // and the couple about that same line (↠)
    const arrows = [];
    let axisCount = 0;
    if (show.wrench) {
      sim.config.wrenches.forEach((w) => {
        if (!w.enabled) return;
        const F0 = sim.wrenchInSpace(w, snap.T0);
        const applied = sim.config.wrenchSense === 'applied' ? F0 : F0.map((x) => -x);
        const sc = wrenchToScrew(applied);
        const T = snap.T0[w.link], origin = [T[0][3], T[1][3], T[2][3]];
        if (sc.kind === 'force') {
          const along = sc.sHat.reduce((s, x, k) => s + x * (origin[k] - sc.q[k]), 0);
          const at = sc.q.map((x, k) => x + sc.sHat[k] * along); // point of the line nearest the target frame
          arrows.push({ from: at, vec: sc.sHat.map((x) => x * sc.magnitude * SCALE.force), color: colors.wrench, label: `${fmt(sc.magnitude, 1)} N` });
          const couple = sc.h * sc.magnitude;
          if (Math.abs(couple) > 1e-6) arrows.push({ from: at, vec: sc.sHat.map((x) => x * couple * SCALE.moment), color: colors.wrench, double: true, label: `${fmt(Math.abs(couple), 2)} N·m` });
          scene.setLine(`wrenchAxis${axisCount++}`, [at.map((x, k) => x - sc.sHat[k] * 0.5), at.map((x, k) => x + sc.sHat[k] * 0.5)], { color: colors.wrench, width: 1.2, dashed: true, opacity: 0.7 });
        } else if (sc.kind === 'couple') {
          arrows.push({ from: origin, vec: sc.sHat.map((x) => x * sc.magnitude * SCALE.moment), color: colors.wrench, double: true, label: `${fmt(sc.magnitude, 2)} N·m` });
        }
      });
    }
    for (let i = axisCount; i < 4; i++) scene.setLine(`wrenchAxis${i}`, null);
    scene.setArrows('wrench', arrows);

    // bearing forces: the part of each joint wrench the motor does not supply
    const bearing = [];
    if (show.bearing) {
      snap.joints.forEach((jw, i) => {
        const T = snap.T0[i], f = jw.bearing.slice(3);
        const f0 = [0, 1, 2].map((r) => T[r][0] * f[0] + T[r][1] * f[1] + T[r][2] * f[2]);
        bearing.push({ from: geom.jointPoints[i], vec: f0.map((x) => x * SCALE.bearing), color: colors.bearing, label: `${fmt(jw.forceMagnitude, 0)} N` });
      });
    }
    scene.setArrows('bearing', bearing);
    this.drawEllipsoids(snap.Jp, snap.p, show.ellipsoids);
  }

  drawAxes(geom, visible) {
    const colors = PALETTE[this.view.theme];
    this.scene.setAxes(visible ? geom.jointPoints.map((p, i) => ({ point: p, dir: geom.jointAxes[i], color: colors.joints[i % 3], label: `S${i + 1}` })) : []);
  }

  drawEllipsoids(Jp, center, visible) {
    const colors = PALETTE[this.view.theme];
    if (!visible) { this.scene.setEllipsoid('manip', null); this.scene.setEllipsoid('force', null); return; }
    const e = pointEllipsoids(Jp);
    this.scene.setEllipsoid('manip', { center, axes: e.axes, radii: e.manipulability.map((r) => Math.max(r * SCALE.manip, 1e-4)), color: colors.manip });
    // near a singularity one force axis grows without bound: clamp what is drawn
    this.scene.setEllipsoid('force', { center, axes: e.axes, radii: e.force.map((r) => Math.min(r, 60) * SCALE.forceEll), color: colors.force });
  }

  // ---------- charts ----------
  updateCharts() {
    const log = this.sim.log, N = this.sim.N, k = this.view.units === 'deg' ? 1 / DEG : 1;
    const times = log.map((r) => r.t);
    const cols = {};
    for (let i = 0; i < N; i++) {
      cols[`th${i}`] = log.map((r) => r.theta[i] * k);
      cols[`thd${i}`] = log.map((r) => r.thetad[i] * k);
      cols[`e${i}`] = log.map((r) => r.e[i] * k);
      cols[`b${i}`] = log.map((r) => r.bearingForce[i]);
    }
    const j = Math.min(this.view.torqueJoint, N - 1);
    for (const key of ['tau', 'inertial', 'c', 'g', 'ext', 'friction']) cols[key] = log.map((r) => r[key][j]);
    for (const key of ['E', 'K', 'P', 'balance']) cols[key] = log.map((r) => r[key]);
    for (const c of Object.values(this.charts)) c.setData(times, cols, this.view.plotWindow);
  }
}

// Re-export for panels that rebuild the robot preview.
export { buildRobot };
