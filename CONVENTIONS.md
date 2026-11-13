# CONVENTIONS

Symbols, frames, units, sources, and every place the two sources disagree.
Sources are cited by section/equation and described in my own words:

- **Notes** = RBE 501 course lecture notes (WPI), cited as "Topic N p. X".
- **MR** = Lynch & Park, *Modern Robotics* (Cambridge, 2017), cited by
  section and equation number.

Rule used throughout: **notation follows the notes; where the notes are silent,
MR; where either has a slip, the math that passes a numerical check wins and
the slip is logged here** (section 6).

---

## 1. Units, frames, ordering

| Item | Convention |
|---|---|
| Units | SI everywhere in the engine: m, kg, s, rad, N, N·m. The UI may show degrees; conversion happens only in `src/ui/`. |
| Space frame `{0}` (= `{s}`) | Fixed at the base, **z up**. Gravity vector `g = (0, 0, -9.81)` m/s². |
| Link frames `{i}` | At each link's centre of mass (Topic 11 p. 11; MR §8.3.1). Orientation is free; the default arm deliberately rotates `{2}` and `{3}` so nothing depends on axis-aligned frames. |
| Tool frame `{N+1}` | End-effector frame, rigidly attached to link N. |
| Twist | `V = (ω, v)`, angular part on top (notes and MR agree). |
| Wrench | `F = (τ, f)`, moment on top, force below (Topic 10 p. 1). MR writes `(m, f)` (MR Eq. 3.93). |
| Joint variable | The notes write θ. Code uses `theta`, `dtheta`, `ddtheta`. |
| Matrices in code | Plain nested arrays `A[row][col]`; vectors are plain arrays. Indices are 0-based in code, 1-based in the maths and in the equation editor. |

## 2. Notation map (notes vs MR vs code)

| Meaning | Notes | MR | Code |
|---|---|---|---|
| Unit screw-axis direction | `ŝ` | `ω̂` | `sHat` |
| Point on the axis | `q` | `q` | `q` |
| Screw axis in `{0}` | `S_i = (ŝ, -ŝ×q + hŝ)` | `S_i` | `S[i]` |
| Screw axis of joint i in link frame `{i}` | `A_i = Ad_{M_i⁻¹} S_i` (Topic 11 p. 11) | `A_i` (MR §8.3.1) | `A[i]` |
| 3×3 skew matrix | `[ω]` | `[ω]` | `skew(w)` |
| 4×4 twist matrix | `[V]` | `[V]` | `bracket6(V)` |
| Big adjoint (of SE(3)) | `Ad_T` (no brackets) | `[Ad_T]` | `adjoint(T)` / `Ad(T)` in the editor |
| Little adjoint (Lie bracket) | `ad_V` | `[ad_V]` (MR Eq. 8.38) | `ad(V)` |
| Spatial inertia | `G_i` | `G_i` (MR Eq. 8.32) | `G[i]` |
| Home pose of `{i}` in `{0}` | `M_i` | `M_i` | `Mlink[i]` |
| Mass matrix | `M(θ)` | `M(θ)` | `massMatrix()` returns `Mq` to avoid the clash with home poses |
| Velocity + gravity torques | `h(θ, θ̇)` | `h` or `c + g` (MR §8.4) | `h = c + g` |
| Column-selector matrix | `D_i` (Topic 13 p. 3) | not in MR | handled by zeroing columns, see `externalTorques()` |
| External wrench on link i | `F^ext_{0,i}` (Topic 11 p. 12) | only `F_tip` | `FextNotes[i]` (see section 4) |

## 3. Equations implemented, with sources

### Kinematics (MR equation numbers verified against the text; notes cited at topic level, see section 7)
- Skew matrix `[ω]` (MR Eq. 3.30); Rodrigues' formula (MR Eq. 3.51); `T⁻¹`
  (MR Eq. 3.64); `e^{[S]θ}` (MR Eq. 3.88); adjoint `Ad_T` (MR Def. 3.20).
- Space-form product of exponentials `T(θ) = e^{[S_1]θ_1}…e^{[S_N]θ_N} M`
  (Topic 4; MR Eq. 4.14).
- Space Jacobian, column `i = Ad_{e^{[S_1]θ_1}…e^{[S_{i-1}]θ_{i-1}}} S_i`
  (Topic 5; MR Def. 5.1, Eq. 5.11).
- Body Jacobian (Topic 6; MR Eq. 5.18) and `J_b = Ad_{T⁻¹} J_s` (MR Eq. 5.22).
- Manipulability measures μ₁ = √(λmax/λmin), μ₂ = λmax/λmin, μ₃ = √det(JJᵀ)
  (Topic 7; MR §5.4). Singularities (Topic 7; MR §5.3).
- Straight-line joint path `θ(s) = θ_start + s(θ_end − θ_start)` (MR Eq. 9.3)
  with cubic and quintic time scaling (Topic 12; MR §9.2.2.1, Eqs. 9.9–9.11).

### Wrenches and statics (Topic 10; MR §3.4, §5.2, §5.4)
- Wrench as a screw: finite pitch `F = f·(-ŝ×q + hŝ, ŝ)`; infinite pitch (pure
  couple) `F = τ·(ŝ, 0)` (Topic 10 pp. 2–3). **Notes only**; MR gives no pitch
  form for wrenches. Poinsot's theorem (Topic 10 p. 1) is also notes-only.
  Note the pitch meaning flips relative to twists: for a wrench `h = 0` is a
  pure force and `h = ∞` a pure couple.
- Power pairing `P = Vᵀ F` (Topic 10 p. 3; MR Eq. 3.94).
- Reciprocal screws: `Vᵀ F = 0` (Topic 10 p. 3). MR uses the word only for
  contacts (§12.1); the manipulator-level statement is the notes'.
- Wrenches reacted with zero joint torque: `null(Jᵀ)` (Topic 10 p. 5; MR §5.2
  discusses `Null(Jᵀ)` without the word "reciprocal").
- Joint-screw table: a revolute joint moves with a zero-pitch twist and its
  motor applies an infinite-pitch wrench `τ(ŝ, 0)`; the other five wrench
  dimensions are reacted passively (Topic 10 p. 6). Used for the motor/bearing
  split.
- Wrench change of frame: `F_b = Ad_{T_ab}ᵀ F_a` (Topic 10 p. 7; MR Eq. 3.95,
  Prop. 3.27). Same `T_ab` as the twist rule `V_a = Ad_{T_ab} V_b`, opposite
  direction.
- Statics `τ = Jᵀ F` with `F = -F_ext` (Topic 10 pp. 7–8; MR Eq. 5.26).
- Twist (manipulability) ellipsoid from `J Jᵀ`; force ellipsoid from
  `(J Jᵀ)⁻¹`, same axes, reciprocal lengths (Topic 10 pp. 8–9 as corrected in
  class; MR §5.4).

### Single rigid body and RNE (Topic 11; MR §8.2–8.3)
- `G_b = diag(I_b, m·1)` at the CoM (Topic 11 p. 5; MR Eq. 8.32).
- Shape inertias for box, cylinder, ellipsoid (Topic 11 p. 5; MR Fig. 8.5).
- `F_b = G_b V̇_b − ad_{V_b}ᵀ G_b V_b` (Topic 11 p. 6; MR Eq. 8.40).
- Inertia in another frame `G_a = Ad_{T_ba}ᵀ G_b Ad_{T_ba}` (Topic 11 p. 7;
  MR Eq. 8.42).
- Setup: `M_{i,i-1} = M_i⁻¹ M_{i-1}`, `A_i = Ad_{M_i⁻¹} S_i`,
  `T_{i,i-1}(θ_i) = e^{-[A_i]θ_i} M_{i,i-1}` (Topic 11 pp. 11–12; MR §8.3.1,
  Eq. 8.50).
- Forward pass: `V_i = A_i θ̇_i + Ad_{T_{i,i-1}} V_{i-1}`,
  `V̇_i = A_i θ̈_i + ad_{V_i} A_i θ̇_i + Ad_{T_{i,i-1}} V̇_{i-1}`
  (Topic 11 pp. 12–13; MR Eqs. 8.51–8.52).
- Backward pass: `F_i = G_i V̇_i − ad_{V_i}ᵀ G_i V_i + Ad_{T_{i+1,i}}ᵀ F_{i+1}
  + Ad_{T_{0,i}}ᵀ F^ext_{0,i}`, `τ_i = F_iᵀ A_i` (Topic 11 p. 13; MR Eqs.
  8.53–8.54 without the last term). The per-link external wrench is
  **notes-only**; MR has only a tip wrench.
- Gravity as a base acceleration `V̇_0 = (0, −g)` with `g` the gravity
  *vector* (Topic 11 p. 14; MR §8.3.2).

### Configuration-space dynamics (Topic 13; MR §8.1, §8.4–8.5)
- `τ = M(θ)θ̈ + h(θ,θ̇) + Σ_{i=1}^{N+1} D_i J_sᵀ F^ext_{0,i}` (Topic 13 p. 3).
  `D_i` and per-link wrenches are **notes-only** (MR Eq. 8.81 has `Jᵀ F_tip`).
- `h = RNE(θ, θ̇, θ̈=0, F^ext=0)` (Topic 13 p. 4; MR §8.5).
- `M(:, i) = RNE(θ, θ̇=0, θ̈=e_i, F^ext=0, g=0)` (Topic 13 pp. 4–5; MR §8.5).
- `M = Σ D_i J_sᵀ Ad_{T_0i⁻¹}ᵀ G_i Ad_{T_0i⁻¹} J_s D_i` from kinetic energy
  (Topic 13 p. 5); MR's equivalent is `Σ J_ibᵀ G_i J_ib` (MR Eq. 8.57).
- Forward dynamics `θ̈ = M⁻¹(τ − h − Σ D_i J_sᵀ F^ext)` (Topic 13 p. 7; MR
  Eq. 8.81). The notes present Euler integration (Topic 13 pp. 7–8) and name
  RK4 as the common better choice (p. 8; MR §8.5); this app uses RK4.
- This code splits `h = c + g` as MR §8.4 does: `g(θ) = RNE(θ, 0, 0)` with
  gravity on, `c(θ,θ̇) = RNE(θ, θ̇, 0)` with gravity off.

### Control (not in the notes; MR Ch. 11)
- Standard second-order error dynamics `ë + 2ζω_n ė + ω_n² e = 0`
  (MR Eq. 11.8), roots MR Eq. 11.10.
- PID joint control (MR Eq. 11.23, §11.4.1.1).
- Computed torque `τ = M̃(θ)(θ̈_d + K_p e + K_i∫e + K_d ė) + h̃(θ,θ̇)`
  (MR Eq. 11.35 single joint, Eq. 11.37 multi-joint, §11.4.2.2).
- PID + gravity compensation (MR Eq. 11.38).
- Gains from `K_p = ω_n²`, `K_d = 2ζω_n` follow from matching MR Eq. 11.33
  (with `K_i = 0`) to MR Eq. 11.8.

## 4. The external-wrench sign convention (read this before touching wrenches)

Two opposite senses exist and mixing them silently breaks the dynamics.

- **Notes' `F^ext_{0,i}`** as it appears in the Topic 11 p. 13 backward pass
  and the Topic 13 p. 3 equation (both with a `+` sign): consistent with MR's
  `F_tip`, i.e. the wrench **link i exerts on the world**. With this sense a
  positive wrench *costs* joint torque: `τ = … + J_sᵀ F^ext`.
- **Applied wrench** `W`: what a person means by "I push on the link with W".
  This is the wrench the world exerts **on** the link. `W = −F^ext`.

The engine (`src/dynamics/`) always takes the **notes' sense** (`FextNotes`),
so its formulas read exactly like Topic 11/13. The UI has a labelled toggle
(default: *applied to the link*) and converts with `FextNotes = −W` in one
place only: `appliedToNotes()` in `src/wrench/wrench.js`.

The notes README records that a Lecture 19 board derivation used the opposite
sense for `F_i` as well; both are self-consistent. This code uses the written
Topic 11 p. 13 form (the MR form): `F_i` is the wrench transmitted **to link i
through joint i** from link i−1.

All wrenches entering the engine are expressed in `{0}`. A wrench typed in
another frame is first moved with `F_0 = Ad_{T_{i0}}ᵀ F_i` (Topic 10 p. 7).

## 5. Things that exist only in the notes (never cite MR for these)
Per-link external wrenches in RNE; `D_i`; the wrench Plücker/pitch form;
Poinsot's theorem; reciprocal screws at the manipulator level.

## 6. Conflicts and slips, and what this code does

| # | Where | Issue | Resolution in code | Evidence |
|---|---|---|---|---|
| 1 | Topic 10 p. 2 | Three solved balance lines drop a minus sign (flagged in the notes' README, never corrected in class). | Not used directly. The wrench transformation is taken from Topic 10 p. 7 / MR Eq. 3.95, which is unaffected. | `tests/wrench.test.js` power-invariance check |
| 2 | Topic 10 p. 9 | Force ellipsoid labelled with `JᵀJ`; corrected in class to `(JJᵀ)⁻¹`, which is also MR §5.4. | `(J Jᵀ)⁻¹`. | `tests/wrench.test.js` ellipsoid duality |
| 3 | Topic 11 p. 5 | Velocity-product matrix written `diag([ω],[v])`; MR Eq. 8.29 has `diag([ω],[ω])`. | Reference uses `−ad_Vᵀ G V` (Topic 11 p. 6 boxed form = MR Eq. 8.40). The p. 5 form is offered as a *deliberately wrong* edit in experiment "Break an equation" so its effect (energy not conserved) can be seen. | `tests/editor.test.js`, experiment E9 |
| 4 | Topic 11 pp. 7, 9, 10 | Missing transpose on `ad` in the worked example. | Transpose kept (MR Eq. 8.43). The example's geometry and numbers are not used anywhere. | `tests/dynamics.test.js` single-body point-mass check |
| 5 | Topic 11 p. 13 | Sign sense of `F_i` / `F^ext` (written form vs a board derivation). | Written p. 13 form; see section 4. | `tests/wrench.test.js` `J_sᵀF` vs RNE |
| 6 | Topic 11 p. 13 vs p. 10 | Backward loop written from `N+1` on one page and `N` on another. | Loop runs `i = N … 1` with `F_{N+1}` initialised to the tip wrench moved into `{N+1}`; identical maths. | same |
| 7 | Topic 11 p. 14 | "−g" as the base acceleration: sign depends on whether `g` is signed. Resolved in the notes' README as MR's `V̇_0 = (0, −g⃗)`. | `V̇_0 = (0, −g⃗)`, base accelerates upward when gravity points down. | `tests/dynamics.test.js` `g(θ)` vs `∂P/∂θ` |
| 8 | Topic 13 pp. 3, 4, 7 | Sum runs to `N+1` but `D_i` is written for `i ≤ N`. | `D_{N+1} = I`: a tip wrench reaches every joint. | `tests/wrench.test.js` |
| 9 | Topic 14 pp. 3, 7 | Two slips in worked examples (README). | Topic 14 is used only as the idea "Christoffel symbols give `c`", implemented independently as a cross-check; neither example is used. | `tests/dynamics.test.js` Christoffel check |

Each row was re-checked numerically in this repo's test-suite rather than
taken on trust; see HANDOFF.md for the numbers of the last run.

## 7. What was and was not read (session 1)

Read in full with `project_read`: the notes' `README.md`, `Lecture10`
(Topic 10), `Lecture11` (Topic 11), `Lecture13` (Topic 13). Page-level
citations to those topics come from that reading.

**Not read:** `Lecture04`–`Lecture07`, `Lecture12`, `Lecture14`. Citations to
Topics 4–7, 12 and 14 are therefore topic-level only and rest on the notes
README's topic map; the equations themselves were taken from MR (numbers
verified against the MR text) and checked numerically. Next agent: read those
files, upgrade the citations to page level, and confirm the notes' own
definitions (especially μ₁, μ₂, μ₃ in Topic 7 and the time scalings in
Topic 12) match what the code does. Log any difference in section 6.

MR was available as extracted text; equation and section numbers cited here
were each found in that text. MR page numbers are not cited.
