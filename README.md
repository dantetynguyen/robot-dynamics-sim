# Robot Dynamics Simulator

An interactive, in-browser simulator of a 3-DOF spatial robot arm, built on
screw theory. You can orbit the arm in 3D, watch where it is predicted to go
before it gets there, apply wrenches, switch controllers, and **edit the
equations of motion as text while the simulation runs** to see what each term
does.

- **Kinematics:** product-of-exponentials forward kinematics, space and body Jacobians, manipulability.
- **Dynamics:** recursive Newton–Euler inverse dynamics; M(θ), c, g and forward dynamics built from it; RK4 integration.
- **Wrenches:** screws, frame changes, power, statics, reciprocal wrenches, force ellipsoid, motor-versus-bearing loads.
- **Control:** PD, PD + gravity compensation and computed torque, run against a controller model you can make wrong on purpose.
- **Verified:** 37 automated checks; kinematics and dynamics agree with an independent numpy implementation to 10⁻¹³ ([ACCEPTANCE.md](ACCEPTANCE.md)).

*(screenshot placeholder: add `docs/screenshot.png`)*

No build step, no server code, no runtime dependencies: plain JavaScript modules.

## Run it

```bash
git clone https://github.com/dantetynguyen/robot-dynamics-sim.git
cd robot-dynamics-sim
python3 -m http.server 8080     # then open http://localhost:8080/
```
(It must be served over http; browsers do not load modules from `file://`.)

```bash
npm test             # 37 acceptance tests, Node 20+, nothing to install
npm run acceptance   # the same, plus a table of every measured number
npm run bench        # simulated seconds per wall-clock second
npm run smoke        # browser smoke test (needs Playwright for Python + Chrome)
```

## Using it

Two modes, switched in the top bar.

**Kinematics** has no physics. Drag the joint sliders and see the screw axes,
the pose assembling one exponential at a time, each joint's share of the tool
velocity, and the manipulability and force ellipsoids.

**Dynamics** is the simulator. Four lines trace the tool point:

| Line | Meaning |
|---|---|
| dashed, pale | desired path (the reference trajectory) |
| blue | predicted path: the true plant simulated forward from now |
| violet, dashed | the controller's belief: its own model simulated forward from now |
| orange | actual path so far |

The gap between blue and violet is the controller's model error, visible
before it plays out. Tabs: **Control** (law, gains, model error, trajectory),
**Wrenches**, **Equations** (the editor), **Learn** (guided experiments),
**Checks** (physical consistency of whatever equations are running), **Robot**
(every parameter of the arm).

## Experiments (Learn tab)

| Experiment | What it shows |
|---|---|
| Motors off: energy is conserved | forward dynamics and integration error |
| Computed torque with a perfect model | every joint error becomes the same second-order system |
| Forget gravity in the model | steady-state droop |
| A wrong model | predicted and belief paths split |
| Push where the arm cannot feel it | wrenches in null(J_sᵀ) cost no torque |
| One wrench, three frames | components change, power and torques do not |
| Spin the waist | bearing load grows with speed squared while motor torque does not |
| Break an equation and read the checks | which property each term protects (and one that energy conservation cannot see) |
| Edit only the controller's model | plant versus model |
| Build the pose one exponential at a time | product of exponentials |
| Jacobian columns are the joint screws | space Jacobian |
| Near a singularity | velocity and force ellipsoids are dual |

## Equations implemented

Notation and sources are in [CONVENTIONS.md](CONVENTIONS.md). Sources: RBE 501
course lecture notes (WPI), and Lynch & Park, *Modern Robotics* (MR).

| Equation | Source |
|---|---|
| T(θ) = e^{[S₁]θ₁} … e^{[S_N]θ_N} M | MR Eq. 4.14 |
| J_s column i = Ad_{e^{[S₁]θ₁}…e^{[S_{i−1}]θ_{i−1}}} S_i; J_b = Ad_{T⁻¹} J_s | MR Eqs. 5.11, 5.22 |
| V_i = A_i θ̇_i + Ad_{T_{i,i−1}} V_{i−1} | notes Topic 11 p. 12; MR Eq. 8.51 |
| V̇_i = A_i θ̈_i + ad_{V_i} A_i θ̇_i + Ad_{T_{i,i−1}} V̇_{i−1} | Topic 11 p. 13; MR Eq. 8.52 |
| F_i = G_i V̇_i − ad_{V_i}ᵀ G_i V_i + Ad_{T_{i+1,i}}ᵀ F_{i+1} + Ad_{T_{0,i}}ᵀ F^ext_{0,i}; τ_i = F_iᵀ A_i | Topic 11 p. 13; MR Eqs. 8.53–8.54 (per-link F^ext: notes only) |
| τ = M θ̈ + h + Σ D_i J_sᵀ F^ext_{0,i}; θ̈ = M⁻¹(τ − h − …) | Topic 13 pp. 3, 7; MR Eq. 8.81 |
| M columns and h from RNE calls; M = Σ J_iᵀ G_i J_i | Topic 13 pp. 4–5; MR §8.5, Eq. 8.57 |
| F_b = Ad_{T_ab}ᵀ F_a; P = VᵀF; τ = JᵀF | Topic 10 pp. 3, 7, 8; MR Eqs. 3.94, 3.95, 5.26 |
| Wrench screw F = f(−ŝ×q + hŝ, ŝ); reciprocal wrenches null(Jᵀ) | Topic 10 pp. 2–5 (notes only) |
| Force ellipsoid from (JJᵀ)⁻¹ | Topic 10 p. 9 as corrected in class; MR §5.4 |
| Computed torque τ = M̂(θ̈_d + K_p e + K_i∫e + K_d ė) + ĥ | MR Eq. 11.37 |

## For developers

[ARCHITECTURE.md](ARCHITECTURE.md) (layout and how to extend),
[CONVENTIONS.md](CONVENTIONS.md) (symbols, frames, sign conventions, source
conflicts), [DECISIONS.md](DECISIONS.md), [HANDOFF.md](HANDOFF.md) (current
state, known issues, next steps), [PROGRESS.md](PROGRESS.md) (build log).

Built by Dante Nguyen with Claude (Anthropic) writing the code from his course
notes and specification; see the commit history. Vendored libraries keep their
own licences (`vendor/*/LICENSE`).
