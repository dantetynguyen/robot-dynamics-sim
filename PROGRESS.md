# PROGRESS log

Running log of the build, newest entries at the bottom. A fresh agent (or a
context-compacted one) should read this file first, then HANDOFF.md.

## Session 1 (2026-11-13) — initial framework build

### Locked decisions (from the Q&A with Dante; do not re-ask)
- HW3 not yet submitted. Build anyway; Dante will not open the code until he
  has submitted. Never open `HW/`, `Midterm/`, `Graded Midterm.pdf`,
  `claude/HW2_Tutor_Key.md`, any `.p`/`.zip` in the claude.ai Project.
- Repo: `dantetynguyen/robot-dynamics-sim`, branch `main`, PUBLIC (his choice).
  Push after each milestone once tests pass. At lock time pushes were refused
  (403, Claude GitHub App not linked). If still refused: do not loop; send a
  git bundle to the chat at each milestone instead.
- External wrench sign: switchable toggle, default "applied TO the link";
  engine converts to the notes' F^ext (= wrench the link exerts on the world).
- Arm: tabletop cobot RRR (waist yaw, shoulder pitch, elbow pitch), own numbers.
- Emphasis: teaching depth first. Theme: dark with light toggle.
- Priorities (all four wanted): predicted-vs-actual paths, reciprocal wrenches,
  motor/bearing split, force/manipulability ellipsoids.
- Two modes: **Kinematics** (sliders, screw axes, PoE build-up, Jacobian
  columns, twists, manipulability; no physics) and **Dynamics** (simulator).
- Defaults: degrees in UI with rad toggle; MATLAB-like equation syntax via a
  whitelisted parser (never eval / new Function); wn = 8 rad/s, zeta = 1, Ki off;
  RK4 dt = 1 ms; prediction horizon 1.5 s; plain ES modules, no build step,
  vendored pinned libs; Node tests + independent numpy reference; private
  preview artifact; notes' notation wins, conflicts logged in CONVENTIONS.md.
- Build order: math core + tests -> sim + controllers -> 3D view + paths ->
  wrench tools -> equation editor -> teaching layer -> docs.

### Environment facts
- Node 22, Python 3.13 + numpy/scipy, npm reachable, headless Chrome +
  playwright (python) available for load checks. CDNs not reachable.
- MR textbook text (extracted) was read via project_read into a temp file;
  cite by section/equation.

### Log
- Phase 0/1 complete, decisions locked. Starting Phase 2.
- Milestone 1 done and pushed (pushes work: Dante linked the GitHub App).
  Math core + numpy reference + 22 tests, all passing; JS vs Python agree to
  ~1e-13 on FK, J_s, M, c, g for the default arm and a random 3R arm.
- Milestone 2 done: whitelisted equation parser/compiler (src/equations/),
  15 editable slots, EquationSet (plant / controller model / shared),
  trajectories, controllers, Simulation class (RK4, prediction, snapshot,
  save/load, fault handling), live checks, benchmark. 37 tests pass.
- FINDING to report to Dante: his example "type the Topic 11 p. 5
  diag([w],[v]) term and watch energy stop being conserved" is not what
  happens. The dropped force m w x v is perpendicular to v, so it does no work;
  energy stays conserved (3.8e-9 J after 2 s) while the motion is wrong and the
  Christoffel check fails (1.97 N·m). Same for a missing transpose on ad.
  Deleting the Coriolis term from the ACCELERATION recursion does break energy
  (4.0 J after 2 s). Tests in tests/editor.test.js. Use this in experiment E9.
- Benchmark (Node 22, this container): 8.7x real time plain; 3.3x with both
  predictions at 5 Hz; 1.9x with one edited equation + predictions; 0.5x with
  all 13 equations edited + predictions (1.2x without predictions).
- hwflag boxes in Lecture11/13 mention HW3 function names and wording; set
  aside, not followed. Mention in final report.
- NEXT: milestone 3 = UI (index.html, three.js scene, panels, plots), then
  teaching layer, docs, delivery. Still to read: Lecture04-07, Lecture12 for
  kinematics/trajectory citations (CONVENTIONS.md section 7).
- (resumed after a usage-limit pause) Vendored pinned libs into vendor/:
  three 0.180.0 (module + core + OrbitControls + fat-line addons), KaTeX
  0.16.22 (js, css, woff2 fonts), IBM Plex Sans latin 400/500/600 woff2.
  Addons import the bare specifier 'three' -> index.html needs an importmap.
- UI design decisions: "lab instrument" look, dark slate default (#151c26
  chart surface) with a light theme; IBM Plex Sans; chart/series colours from
  the validated dataviz palette (dark: blue #3987e5, orange #d95926, aqua
  #199e70, yellow #c98500, magenta #d55181, violet #9085e9, red #e66767).
  Joints = aqua/yellow/magenta. Paths: desired = neutral dashed, predicted =
  blue, belief = violet, actual = orange. Wrench = red.
- Kinematics-file reading (Lecture04-07, 12) deferred until the app works;
  until then kinematics cites MR equations (verified by grep) + notes topics.
- Milestone 3+4 done and pushed: full UI (two modes, 3D view, charts, six
  tabs), 12 experiments, teaching text. Smoke test (tools/smoke_test.py)
  passes: every tab and experiment runs with no page errors.
- Usage budget ran low: owner re-prioritised to (1) working UI, (2) teaching
  layer, (3) HANDOFF.md. Docs written: HANDOFF, ARCHITECTURE, DECISIONS,
  README, ACCEPTANCE, CONVENTIONS section 7 (what was not read).
- NOT done: Lecture04-07/12 reading (kinematics citations are topic-level),
  real-GPU performance measurement, screenshot. See HANDOFF.md sections 6, 8.
