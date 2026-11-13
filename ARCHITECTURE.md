# ARCHITECTURE

A static site: `index.html` loads ES modules straight from `src/`. No bundler,
no build step, no runtime npm dependencies. The same modules run in Node for
the tests. Everything below `src/ui/` needs a browser; nothing else does.

```
index.html, css/style.css      page shell and styles (design tokens at the top of the CSS)
vendor/                        pinned three.js, KaTeX, IBM Plex Sans (no CDN needed)
src/
  math/linalg.js               plain-array matrices: products, solve, Jacobi eigen-solver
  math/lie.js                  skew, matrixExp3/6, adjoint (Ad_T), ad (ad_V), transInv, screwAxis
  robot/model.js               spec (JSON) -> robot: S_i, M_i, M_{i,i-1}, A_i, G_i; validation
  robot/inertia.js             shape inertias, spatial inertia G
  robot/presets.js             the default RRR arm (add robots here)
  dynamics/kinematics.js       fkSpace, jacobianSpace, jacobianBody, pointJacobian
  dynamics/rne.js              recursive Newton–Euler as five swappable equations
  dynamics/terms.js            M(θ), c, g, external torques, forward dynamics, energy
  wrench/wrench.js             frame change, screw form, power, statics, null(Jᵀ), ellipsoids, motor/bearing split
  control/controllers.js       controller list, gains from ωn/ζ, law evaluation on the hat model
  control/trajectory.js        hold, cubic, quintic, sinusoid references
  equations/parser.js          text -> syntax tree (no eval anywhere)
  equations/compile.js         syntax tree -> closure over a function whitelist
  equations/registry.js        the 15 editable equation slots + EquationSet (validation, status)
  sim/integrator.js            RK4 and Euler steps
  sim/simulation.js            Simulation: plant + hat model + controller + RK4 + prediction + save/load
  sim/checks.js                live physics checks (symmetry, Christoffel, energy balance, …)
  teach/experiments.js         the guided experiments (data only)
  ui/app.js                    App: top bar, tabs, frame loop, scene and chart updates
  ui/scene.js                  three.js view: arm, paths, arrows, axes, ellipsoids, labels
  ui/plots.js                  canvas strip charts
  ui/panel_*.js                one file per tab
  ui/geometry.js, dom.js, format.js, setup_io.js   helpers
reference/generate_fixtures.py independent numpy implementation -> tests/fixtures/reference.json
tests/*.test.js                acceptance tests (node --test); helpers.js records every measured number
tools/                         export_robot, acceptance_report, benchmark, smoke_test.py
```

## Data flow

```
spec (JSON) --buildRobot--> robot --+
                                     +--> Simulation.derivative(t, x)  --rk4Step-->  x(t+dt)
EquationSet (plant / model / shared)-+         |  controlTorque(law, hat model)
                                               |  modelAcceleration(plant)  =  M⁻¹(τ − bias − friction)
App.frame():  sim.step(n) -> sim.snapshot() -> Scene3D / StripChart / panel.refresh()
```

State vector: `x = [θ, θ̇, ∫e dt, W]` (`W` = net work, for the energy-balance check).
Two dynamics models exist at all times: `sim.plant` (true robot) and `sim.hat`
(the controller's model). Each is `{ robot, eqs, gravityOn, coriolisOn, wrenchOn }`.

## How to extend

| To add… | Do this |
|---|---|
| a controller | add its law to `controllerSlots()` in `equations/registry.js`, add one line to `CONTROLLERS` in `control/controllers.js` |
| an editable equation | add a slot to `SLOTS` in `equations/registry.js` (seed text, variables, reference function, sampler) and call `equationSet.fn(id)(scope)` where it is used; editor, validation, save/load and tests pick it up |
| a function usable in equations | add it to `FUNCTIONS` in `equations/compile.js` (argument count + implementation) |
| a robot | add a spec-returning function to `robot/presets.js`; the engine is N-general, the UI assumes N = 3 in a few labels and colours |
| a trajectory | add an entry to `TRAJECTORIES` in `control/trajectory.js` |
| a 3D visualization | add a method to `Scene3D` (pattern: `setPath`, `setArrows`, `setEllipsoid`) and call it from `App.updateScene()` |
| a chart | add a `StripChart` in `App.buildCharts()` and its columns in `App.updateCharts()`; log new quantities in `Simulation.record()` |
| a tab | write `ui/panel_x.js` exporting `(app) => ({ el, refresh })`, register it in `PANELS` and `TABS` in `ui/app.js` |
| an experiment | append an object to `EXPERIMENTS` in `teach/experiments.js` |
| a physics check | add a row in `sim/checks.js` |

## Conventions that matter when editing

- Engine units are SI and radians; degrees exist only in `src/ui/` (`app.toUI` / `app.fromUI`).
- Twists `(ω, v)`, wrenches `(τ, f)`; code indices are 0-based, maths and the equation editor are 1-based.
- External wrenches inside the engine use the notes' sense (wrench the link exerts on the world), in `{0}`. See `CONVENTIONS.md` §4.
- Arrays are never mutated after creation (the compiler caches `Ad(T)` by object identity and relies on this).
- `Simulation.step()` never throws; it sets `sim.fault` and returns `false`.
