# DECISIONS

Significant choices and why. Decisions locked with the owner in the kickoff
Q&A are at the top of `PROGRESS.md`; these are the judgment calls made while
building.

| # | Decision | Why |
|---|---|---|
| 1 | Plain ES modules, no framework, no bundler | The brief asks for a static site with no build step that a learner can read. The same files run in Node for tests. |
| 2 | Own small linear algebra on nested arrays instead of a matrix library | Matrices are at most 6×6; `matVec(G, Vd)` reads like the maths; no dependency. |
| 3 | Own parser + whitelist compiler for typed equations instead of vendoring mathjs | Satisfies "never eval" with ~400 readable lines, identical in Node and browser, and is about 5× faster in the inner loop than a general library would be. Product chains ending in a vector are folded right-to-left for speed. |
| 4 | RNE written as five named, swappable equations | Lets the editor replace any step for the true plant, the controller's model, or both, without a second code path. Unedited slots run the built-in functions; edited ones run compiled text. |
| 5 | The controller is evaluated continuously inside each RK4 stage by default | Needed to meet the 1e-6 rad computed-torque check (measured 5e-11). `controlRate > 0` switches to sample-and-hold as a teaching option. |
| 6 | An extra integrated state `W` (net work) and the check `E − E₀ − W = 0` | Works under any controller, not just zero torque, so it is a live correctness check during every experiment. |
| 7 | Energy and the live checks always use the reference inertia/kinematics | Otherwise a wrong edited equation would also corrupt the yardstick it is measured with. |
| 8 | Two predictions drawn: true plant (blue) and controller's belief (violet) | The brief offered both readings of "predicted path"; showing both makes the gap between them the model error. |
| 9 | Prediction step 10 ms, refreshed ≤ 5×/s and less often when expensive | Predictions were costing 6× the simulation itself at 5 ms / 10 Hz. |
| 10 | M(θ) is always built from N RNE calls, even in the integrator | The brief asks for M from RNE. A kinetic-energy fast path would be ~3× quicker; it is kept only as a cross-check. Listed as an optimisation in HANDOFF.md. |
| 11 | Independent reference in Python uses the space frame, no recursion, scipy `expm` | A different derivation in a different language, so agreement to 1e-13 means something. |
| 12 | Tests also run on a random 3R arm with rotated CoM frames and full inertias | The default arm has convenient symmetry that could hide frame errors. |
| 13 | Ellipsoids are drawn for the tool **point** (3×3 `J_p J_pᵀ`), not the 6-D twist | A 3-joint arm's 6×3 Jacobian has a degenerate 6-D ellipsoid; the point version is drawable and is what the notes' pictures show. |
| 14 | Wrench frames offered: space, the target's own frame, "at the target with world axes" | The third is how people think ("15 N down at the tool"); the three together make the frame-change experiment. |
| 15 | Bundles are sent to the chat only at the end | Pushes to GitHub worked from the first milestone, so mid-build bundles would have been noise. |
| 16 | Chart and series colours taken from a palette validated for colour-vision separation on both surfaces; line style (dashed/thin) doubles the colour coding of the paths | Identity must not rest on colour alone. |
| 17 | "Lab instrument" look: slate surfaces, IBM Plex Sans, colour reserved for data | The 3D scene and its path lines are the one loud element. |
| 18 | Kinematics-notes files not read in session 1; kinematics cites MR equations (verified) and notes topics | Usage budget. Flagged in HANDOFF.md §6 and CONVENTIONS.md §7. |
| 19 | Experiment "Break an equation" was redesigned | The brief expected the Topic 11 p. 5 `diag([ω],[v])` term to break energy conservation. Tests show it does not (the dropped force does no work), so the experiment now teaches that instead and offers a second edit that does break energy. |
