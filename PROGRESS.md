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
