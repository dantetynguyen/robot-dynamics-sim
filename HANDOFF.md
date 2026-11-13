# HANDOFF

For the next agent (any model) picking this repo up cold. Read this file,
then `ARCHITECTURE.md`, then `CONVENTIONS.md`. `PROGRESS.md` is the running
log of how the build went; `DECISIONS.md` is why things are the way they are.

Last updated: 2026-11-13, end of build session 1 (Claude Fable 5.2).
The session ended under a usage budget, so polish and some docs were cut;
everything listed as done below was run and checked.

## 1. What this is

A static, no-build web app: an interactive simulator of a 3-DOF spatial RRR
arm using screw theory (product of exponentials), recursive Newton–Euler
dynamics, wrench theory and joint-space control, with the governing equations
editable as text while it runs. Owner: Dante Nguyen (WPI, RBE 501 / ME 528).
Purpose: (1) teach him the course material, (2) portfolio piece.

## 2. Hard rules that still apply (from the owner's brief)

1. **Stay clear of graded coursework.** Do not open, in the claude.ai Project
   or the course folder: anything under `HW/` or `Midterm/`,
   `Graded Midterm.pdf`, `claude/HW2_Tutor_Key.md`, any `.p` or `.zip`. Do not
   follow the notes' `hwflag` boxes. Keep the robot generic with this
   project's own numbers (not an assigned or worked-example robot).
   HW3 (RNE / mass matrix / forward dynamics) was **not yet submitted** on
   2026-11-13; the owner said he will not open this code until it is.
2. **The repo is public.** Cite the notes as "RBE 501 course lecture notes
   (WPI)" by topic and page, in your own words. Never paste notes text,
   slides, transcript quotes or textbook passages/figures.
3. **Accuracy.** Do not invent equations, citations or section numbers. Cite
   only what you actually read. Never silently correct a source: log it in
   `CONVENTIONS.md` §6. Report only test numbers you ran.
4. **User-typed equations never reach `eval` / `new Function`.** They go
   through `src/equations/parser.js` + `compile.js` (whitelist).
5. **Read the Project, do not change it** (`project_read` with exact paths;
   no `project_search`, no writes).
6. Git: push only to `dantetynguyen/robot-dynamics-sim` (`main`). The owner
   asked for a push after every significant change.

## 3. Run, test, verify

```bash
python3 -m http.server 8080        # then open http://localhost:8080/   (ES modules need http, not file://)
npm test                           # 37 Node tests, ~2 s, no dependencies to install
npm run acceptance                 # tests + a table of every measured number (tests/results/ACCEPTANCE.md)
npm run bench                      # real-time ratio in Node
npm run reference                  # regenerate tests/fixtures/reference.json (needs python3 + numpy + scipy)
```

There is no `npm install` step: the app has zero runtime npm dependencies and
the tests use only Node's built-in runner (Node ≥ 20). Vendored browser
libraries live in `vendor/` (three 0.180.0, KaTeX 0.16.22, IBM Plex Sans).

Browser smoke test (needs Playwright for Python and Chrome, and the site
served on port 8080):

```bash
python3 -m http.server 8080 &
npm run smoke                      # = python3 tools/smoke_test.py [url] [screenshot.png]
```
It loads the page, opens every tab, sets up and runs all 12 experiments,
types a syntax error into an equation, switches theme and mode, and exits 1
on any page error, failed request, console error or simulation fault.
`window.app` is the `App` instance, so browser tests can call
`app.setMode('dynamics', 'wrench')`, `app.setRunning(true)`, and read
`app.sim.t` / `app.sim.fault`.

## 4. Verification status (all measured on 2026-11-13)

Full table: `ACCEPTANCE.md`. Summary:

| Area | Result |
|---|---|
| FK, J_s, M, c, g vs independent numpy reference, 100 random states, default arm and a random 3R arm | worst 4.1e-13 (limit 1e-9) |
| M from RNE vs kinetic-energy formula | 4.3e-14 |
| J_s vs finite-difference FK | 5.7e-10 (limit 1e-6) |
| g vs ∂P/∂θ | 1.7e-8 (limit 1e-6) |
| c vs Christoffel symbols of M | 2.4e-9 (limit 1e-5) |
| M symmetric positive definite at all sampled states | yes; smallest eigenvalue 0.0089 kg·m² |
| Energy drift, zero torque, 10 s, dt = 1 ms | 1.45e-6 J = 5.5e-8 of E₀ (limit 0.1%) |
| Computed torque vs second-order prediction, 5 s, ωn = 8, ζ ∈ {0.5, 1, 1.6} | worst 5.2e-11 rad (limit 1e-6) |
| θ̇ = 0 ⇒ inverse dynamics = g(θ); home hold torques vs hand calculation | 0 and 1.8e-15 N·m; g(home) = [0, 11.046, 1.913] N·m |
| Power invariant across frames | 1.3e-15 W (limit 1e-12) |
| J_sᵀF vs RNE with F^ext (tip and per-link) | 4.3e-14 N·m |
| null(J_sᵀ) wrenches → zero torque / zero acceleration | 7.2e-14 N·m / 4.3e-13 rad/s² |
| Screw/pitch form rebuilds the wrench | 2.2e-15 |
| Static hold with applied wrench ⇒ θ̈ = 0 | 1.9e-13 rad/s² |
| Force/manipulability ellipsoid duality | 1.1e-16 |
| Motor + bearing = F_i, F_iᵀA_i = τ_i | 4.4e-16, 0 |
| Seeded editor text vs reference (15 equations × 100 inputs) | 3.7e-16 relative (limit 1e-12) |
| Known-wrong edit flagged; syntax error does not stop the sim | yes; state identical to an untouched run |
| Site loads with no console errors | yes (headless Chrome 153, software WebGL; see ACCEPTANCE.md) |
| Faster than real time | Node: 8.7x plain, 3.3x with both predictions, 1.9x with one edited equation + predictions, **0.5x with all 13 equations edited + predictions** (1.2x without predictions). Headless Chrome with software rendering: 0.91–0.93x at 1x speed. **Not measured on a real GPU.** |

**Unresolved / not verified:**
- Real-GPU browser performance was not measured.
- The all-equations-edited case is slower than real time with predictions on.
  The UI copes (it lowers the prediction refresh rate and reports the achieved
  speed in the top bar) but the acceptance target is only met for a few edited
  equations, not all thirteen at once.
- The UI was checked for runtime errors and by two screenshots only. No
  cross-browser test, no mobile test, no accessibility audit.
- No private preview artifact was published (see §7).

## 5. What is built

**Engine (complete, tested)** — `src/math`, `src/robot`, `src/dynamics`,
`src/wrench`, `src/control`, `src/sim`, `src/equations`:
PoE forward kinematics; space and body Jacobians; RNE with per-link external
wrenches; M, c, g, forward dynamics; RK4 (and Euler); energy and an
energy-balance state; wrench frame change, screw form, power, statics,
reciprocal basis, point ellipsoids, motor/bearing split; zero / PD /
PD + gravity / computed-torque laws; cubic, quintic, sinusoid, hold
references; controller "hat" model with mass scaling, dropped ĝ/ĉ, wrench
knowledge; prediction on the true plant or on the controller's belief;
fault handling (never throws out of `step()`); save/load; 15 editable
equations with live validation; live physics checks.

**UI (functional, lightly polished)** — `index.html`, `css/style.css`,
`src/ui`: two modes (Kinematics, Dynamics); orbit/zoom 3D view with the arm,
link frames, screw axes, four tool paths (desired dashed, predicted, belief,
actual), future-pose ghosts, wrench screws, bearing-force arrows, ellipsoids,
PoE build-up ghosts, per-joint tool-velocity arrows and the twist's screw
axis; four live charts with hover tooltips; tabs Control, Wrenches, Equations,
Learn, Checks, Robot; degrees/radians; dark/light theme; setup kept in
`localStorage`, downloadable, copyable, loadable.

**Teaching layer (complete as designed, not reviewed by the owner)** —
12 guided experiments (`src/teach/experiments.js`), equation cards with live
numbers, DH bridges and citations in the Learn, Kinematics and Wrenches
panels.

## 6. Not built, or only partly

1. **README.md has no screenshot** (placeholder line only) and was written
   quickly; it has not been read by the owner. No LICENSE file exists for the
   project itself (owner's choice to make).
2. **Kinematics citations to the notes are topic-level only.** The notes files
   `Lecture04`–`Lecture07` and `Lecture12` were **not read** (budget), so
   kinematics and trajectory text cites "Topic 4/5/6/7" and MR equation
   numbers (those were verified against the MR text). Files that were read in
   full: the notes `README.md`, `Lecture10`, `Lecture11`, `Lecture13`.
   `CONVENTIONS.md` §7 is a placeholder for this.
3. No side-by-side comparison of two controllers (an idea from the brief).
4. No colour-coding of paths by error or energy.
5. No table view of chart data; charts are canvas-only.
6. Robot tab edits positions but not link-frame orientations or the tool
   orientation (they are kept from the preset; a saved setup file can change
   them). Changing the number of joints from the UI is not supported, though
   the engine is N-general and tested only for N = 3.
7. Prismatic joints: `matrixExp6`, `screwAxis` and RNE handle them, but
   `splitJointWrench` (motor/bearing) assumes revolute and nothing was tested
   with a prismatic joint.
8. Joint limits, torque limits (other than `sat()` in a law), gear/rotor
   inertia, task-space control: none.
9. Second-order prediction overlay on the error chart: not drawn (the test
   checks it numerically instead).
10. GitHub Pages is not switched on (owner action, see §9).

## 7. Known issues and traps

- **`sim.snapshot()` is called every frame and inside `record()`**. It is
  cheap (a few RNE calls) but if you add heavy work to it the frame rate
  drops. Put heavy things in `analysis()` or a panel's `refresh()` (5 Hz).
- **Equation scopes carry both `T` and `AdT`** (and `Tnext`/`AdTnext`):
  the built-in reference functions use the precomputed adjoints, the compiled
  text uses `Ad(T)`. If you add a variable to a slot, add it in three places:
  the call site (`rne.js`), `vars`, and `sample` in `registry.js`.
- **`EquationSet.set()` with text equal to the seed resets to the fast
  built-in path.** To force the compiled path (tests, benchmark) wrap the seed
  in parentheses.
- **Sign convention**: the engine's `Fext` is the notes' F^ext (wrench the link
  exerts on the world). The UI default is the opposite ("applied to the
  link"). The only conversion is `appliedToNotes()`; see `CONVENTIONS.md` §4.
- **A wrench typed in frame "space" is about the {0} origin.** A pure force
  there is a force whose line passes through the base, which surprises
  people; "aligned" (at the target, world axes) is the intuitive default.
- **Energy is always measured with the reference equations** (`robot.Gref`),
  on purpose, so it stays a yardstick when the plant runs an edited equation.
- **Physics finding that contradicts the brief** (keep it, it is tested): the
  Topic 11 p. 5 `diag([ω],[v])` velocity term does **not** break energy
  conservation. It drops the force m ω×v, which is perpendicular to v and so
  does no work (3.8e-9 J balance error after 2 s) while the motion is wrong
  (Christoffel check off by 1.97 N·m). A missing transpose on `ad` is workless
  too. Deleting the Coriolis term from the *acceleration* recursion does break
  energy (4.0 J in 2 s). Experiment "Break an equation" is built on this.
- **Sample-and-hold control** (`controlRate > 0`) is honoured by the real
  simulation but the predictions treat the controller as continuous.
- **three.js addons import the bare name `three`**; `index.html` maps it with
  an import map. Keep that if you move files.
- `tests/results/` is git-ignored; the committed copy of the numbers is
  `ACCEPTANCE.md`. Regenerate both with `npm run acceptance` and copy.
- Reading the notes surfaced `hwflag` boxes naming the HW3 functions and
  paraphrasing their statements (in `Lecture11`, `Lecture13`). They were set
  aside and not followed; the code's API was designed independently.
- A preview artifact was not published. The claude.ai artifact sandbox blocks
  downloads and clipboard unless triggered from a click (the setup panel
  already falls back to a text box) and needs every file uploaded
  individually (~90 files).

## 8. Recommended next steps, in order

1. **Add a screenshot** (`docs/screenshot.png`, referenced from README.md;
   `tools/smoke_test.py url shot.png` can take one) and have the owner pick a
   licence.
2. **Extend `tools/smoke_test.py`** with assertions on what is drawn (e.g.
   path point counts, chart data present, panel numbers finite). Today it
   only proves nothing throws; the UI has no other tests.
3. **Read `Lecture04`–`Lecture07` and `Lecture12`** with `project_read` and
   upgrade the kinematics/trajectory citations from topic-level to page-level
   (panel_kinematics.js, experiments.js, registry.js `traj.*`,
   `CONVENTIONS.md` §7). Check the notes' definitions of μ₁, μ₂, μ₃ against
   `pointEllipsoids()` (currently MR §5.4 definitions).
4. **Measure performance in a real browser** and, if needed, speed up the
   compiled-equation path (ideas: reuse the controller's M̂/ĉ/ĝ when the hat
   model is identical to the plant; compute M columns with a velocity-free
   fast path when the RNE equations are unedited; move predictions to a Web
   Worker).
5. **Owner review of the teaching text** (experiments and notes) for tone and
   for agreement with how the course phrases things.
6. Side-by-side controller comparison (two `Simulation` instances sharing a
   config, second arm drawn as a ghost): the architecture supports it, since
   `Simulation` has no globals.
7. Colour the actual path by tracking error; overlay the analytic
   second-order response on the error chart (`secondOrderResponse()` exists).
8. Robot tab: orientation editing, raw-inertia editing, a second preset
   (e.g. a SCARA-like RRP to exercise prismatic joints, after testing them).
9. Publish a private preview artifact if the owner still wants one.

## 9. Owner actions

```bash
git clone https://github.com/dantetynguyen/robot-dynamics-sim.git
cd robot-dynamics-sim
python3 -m http.server 8080      # open http://localhost:8080/
npm test
```
GitHub Pages: repository → Settings → Pages → Source "Deploy from a branch",
branch `main`, folder `/ (root)` → Save. The site appears at
`https://dantetynguyen.github.io/robot-dynamics-sim/`. No build step.

If the repo ever has to be restored from the bundle sent in the chat:
```bash
git clone robot-dynamics-sim.bundle robot-dynamics-sim
cd robot-dynamics-sim
git remote set-url origin https://github.com/dantetynguyen/robot-dynamics-sim.git
git push -u origin main
```
