// rne.js — inverse dynamics by the recursive Newton–Euler (RNE) algorithm.
//
// Given θ, θ̇, θ̈ it returns the joint torques τ, and on the way the twist,
// acceleration and joint wrench of every link.
//
//   Forward pass (base -> tip): twists and accelerations.
//   Backward pass (tip -> base): wrenches and torques.
//
// Sources: Topic 11 pp. 10–14; MR §8.3 (Eqs. 8.50–8.54). The per-link external
// wrench term is from the notes only (MR has just a tip wrench).
//
// DH bridge: this is the classical outward/inward Newton–Euler recursion.
// There, ω, ω̇ and CoM accelerations are pushed outward with rotation matrices
// and ω×r terms; here one 6x6 adjoint does the rotation and the lever arm
// together, and ad_V collects the ω× (Coriolis/centripetal) products.
//
// SIGN CONVENTION (see CONVENTIONS.md §4): `Fext[i]` is the notes' F^ext_{0,i},
// the wrench link i exerts ON THE WORLD, expressed in {0}. A push applied TO
// the link is the negative of that.

import { matMul, matTVec, matVec, zerosVec } from '../math/linalg.js';
import { ad, adjoint, matrixExp6, transInv } from '../math/lie.js';

// ---------------------------------------------------------------------------
// The five equations of the algorithm, as separate small functions.
// These are the REFERENCE versions. The equation editor can swap any of them
// for a user-typed expression (src/equations/); the scope object `s` carries
// the same variable names the editor shows.
// ---------------------------------------------------------------------------
export const referenceEquations = {
  // Gravity as an acceleration of the base, opposite to gravity:
  //   V̇_0 = (0, −g)   with g the gravity vector       (Topic 11 p. 14; MR §8.3.2)
  baseAccel: (s) => [0, 0, 0, -s.g[0], -s.g[1], -s.g[2]],

  // Twist of link i = this joint's motion + the previous link's twist seen
  // from frame {i}:
  //   V_i = A_i θ̇_i + Ad_{T_{i,i-1}} V_{i-1}          (Topic 11 p. 12; MR Eq. 8.51)
  twist: (s) => {
    const out = matVec(s.AdT, s.Vprev);
    for (let k = 0; k < 6; k++) out[k] += s.A[k] * s.dtheta;
    return out;
  },

  // Acceleration of link i:
  //   V̇_i = A_i θ̈_i + ad_{V_i} A_i θ̇_i + Ad_{T_{i,i-1}} V̇_{i-1}
  //                                                    (Topic 11 p. 13; MR Eq. 8.52)
  // The middle term is the Coriolis/centripetal contribution: it appears
  // because Ad_{T_{i,i-1}} itself changes as joint i turns.
  accel: (s) => {
    const out = matVec(s.AdT, s.Vdprev);
    const cor = matVec(ad(s.V), s.A);
    for (let k = 0; k < 6; k++) out[k] += s.A[k] * s.ddtheta + cor[k] * s.dtheta;
    return out;
  },

  // Wrench transmitted to link i through joint i:
  //   F_i = G_i V̇_i − ad_{V_i}ᵀ G_i V_i            this link's own dynamics
  //       + Ad_{T_{i+1,i}}ᵀ F_{i+1}                 wrench passed on to the next link
  //       + Ad_{T_{0,i}}ᵀ F^ext_{0,i}               wrench this link exerts on the world
  //                                                    (Topic 11 p. 13; MR Eq. 8.53)
  wrench: (s) => {
    const GV = matVec(s.G, s.V);
    const out = matVec(s.G, s.Vd);
    const vel = matTVec(ad(s.V), GV);
    const next = matTVec(s.AdTnext, s.Fnext);
    for (let k = 0; k < 6; k++) out[k] += -vel[k] + next[k];
    if (s.hasFext) {
      const ext = matTVec(adjoint(s.T0i), s.Fext);
      for (let k = 0; k < 6; k++) out[k] += ext[k];
    }
    return out;
  },

  // The motor supplies only the component of F_i along the joint's own screw:
  //   τ_i = F_iᵀ A_i                                   (Topic 11 p. 13; MR Eq. 8.54)
  torque: (s) => {
    let t = 0;
    for (let k = 0; k < 6; k++) t += s.F[k] * s.A[k];
    return t;
  },
};

// ---------------------------------------------------------------------------
// Configuration-dependent transforms, computed once per θ and shared by every
// RNE call at that configuration (M(θ) needs N+1 calls at the same θ).
//
//   Trel[i]  = T_{i,i-1}(θ_i) = e^{-[A_i]θ_i} M_{i,i-1}   (Topic 11 p. 11; MR Eq. 8.50)
//   Trel[N]  = M_{N+1,N}, the fixed tool offset
//   AdRel[i] = Ad_{Trel[i]}
//   T0[i]    = T_{0,i}, pose of frame {i} in {0}; T0[N] is the tool frame
// ---------------------------------------------------------------------------
export function linkKinematics(robot, theta) {
  const N = robot.N;
  const Trel = new Array(N + 1);
  const AdRel = new Array(N + 1);
  const T0 = new Array(N + 1);
  for (let i = 0; i < N; i++) {
    Trel[i] = matMul(matrixExp6(robot.A[i], -theta[i]), robot.Mrel[i]);
    AdRel[i] = adjoint(Trel[i]);
    const Tinv = transInv(Trel[i]); // T_{i-1,i}
    T0[i] = i === 0 ? Tinv : matMul(T0[i - 1], Tinv);
  }
  Trel[N] = robot.Mrel[N];
  AdRel[N] = adjoint(Trel[N]);
  T0[N] = matMul(T0[N - 1], transInv(Trel[N]));
  return { theta: theta.slice(), Trel, AdRel, T0 };
}

// ---------------------------------------------------------------------------
// The algorithm.
//
//   robot    built robot (src/robot/model.js)
//   kin      linkKinematics(robot, θ)
//   dtheta   θ̇
//   ddtheta  θ̈
//   opts.gravity  gravity vector in {0}, or null for no gravity
//   opts.Fext     array of N+1 wrenches in {0} (notes' sense), entry i on
//                 link i, entry N on the tool frame; null entries are skipped
//   opts.eqs      equation set (defaults to referenceEquations)
//
// Returns { tau, V, Vd, F }: joint torques, and per link the body twist V_i,
// acceleration V̇_i and joint wrench F_i, all expressed in the link frame {i}.
// ---------------------------------------------------------------------------
export function rne(robot, kin, dtheta, ddtheta, opts = {}) {
  const N = robot.N;
  const eqs = opts.eqs ?? referenceEquations;
  const Fext = opts.Fext ?? null;

  const V = new Array(N);
  const Vd = new Array(N);
  const F = new Array(N);
  const tau = new Array(N);

  // ----- forward iterations: i = 1 … N -----
  let Vprev = zerosVec(6); // V_0 = 0: the base does not move
  let Vdprev = opts.gravity ? eqs.baseAccel({ g: opts.gravity }) : zerosVec(6);
  for (let i = 0; i < N; i++) {
    V[i] = eqs.twist({
      A: robot.A[i], dtheta: dtheta[i], T: kin.Trel[i], AdT: kin.AdRel[i], Vprev,
    });
    Vd[i] = eqs.accel({
      A: robot.A[i], dtheta: dtheta[i], ddtheta: ddtheta[i], V: V[i],
      T: kin.Trel[i], AdT: kin.AdRel[i], Vdprev,
    });
    Vprev = V[i];
    Vdprev = Vd[i];
  }

  // ----- backward iterations: i = N … 1 -----
  // F_{N+1}: a wrench on the tool frame, moved from {0} into {N+1}
  // (F_b = Ad_{T_ab}ᵀ F_a, Topic 10 p. 7). The tool frame has no mass of its own.
  let Fnext = Fext && Fext[N] ? matTVec(adjoint(kin.T0[N]), Fext[N]) : zerosVec(6);
  for (let i = N - 1; i >= 0; i--) {
    const hasFext = !!(Fext && Fext[i]);
    F[i] = eqs.wrench({
      G: robot.G[i], V: V[i], Vd: Vd[i],
      Tnext: kin.Trel[i + 1], AdTnext: kin.AdRel[i + 1], Fnext,
      T0i: kin.T0[i], Fext: hasFext ? Fext[i] : ZERO6, hasFext,
    });
    tau[i] = eqs.torque({ F: F[i], A: robot.A[i] });
    Fnext = F[i];
  }

  return { tau, V, Vd, F };
}

const ZERO6 = Object.freeze([0, 0, 0, 0, 0, 0]);

// Convenience wrapper when only one call is needed at this θ.
export function inverseDynamics(robot, theta, dtheta, ddtheta, opts = {}) {
  return rne(robot, linkKinematics(robot, theta), dtheta, ddtheta, opts).tau;
}
