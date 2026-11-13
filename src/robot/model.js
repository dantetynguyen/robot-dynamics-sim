// model.js — turns a human-editable robot description ("spec") into the
// constant quantities the kinematics and dynamics need.
//
// SPEC (plain JSON, this is what gets saved/loaded and what the UI edits):
// {
//   name: 'my arm',
//   gravity: [0, 0, -9.81],          // gravity VECTOR in {0}, m/s²
//   joints: [                        // one per joint, base to tip
//     { sHat: [0,0,1], q: [0,0,0], h: 0 }   // axis direction, point, pitch
//     // or  { S: [ω, v] }  to give the screw axis as a raw 6-vector
//   ],
//   links: [                         // link i moves with joint i
//     { mass, Mlink: 4x4,            // home pose of the CoM frame {i} in {0}
//       shape: {type:'cylinder', r, h} | {type:'box',...} | {type:'matrix', I} }
//   ],
//   Mee: 4x4,                        // home pose of the tool frame {N+1}
//   damping: [b1, …],                // viscous joint damping, N·m·s/rad
// }
//
// DERIVED (Topic 11 pp. 11–12; MR §8.3.1):
//   S[i]      screw axis of joint i in {0}
//   Mlink[i]  M_i, home pose of {i} in {0}       (Mlink[N] is the tool frame)
//   Mrel[i]   M_{i,i-1} = M_i⁻¹ M_{i-1}          (with M_0 = I)
//   A[i]      A_i = Ad_{M_i⁻¹} S_i, joint i's screw seen from link frame {i}
//   G[i]      spatial inertia of link i in {i}
//
// Indices are 0-based in code: joint/link "1" of the maths is index 0.

import { eye, matMul, matVec, norm } from '../math/linalg.js';
import { adjoint, screwAxis, transInv } from '../math/lie.js';
import { inertiaProblems, shapeInertia, spatialInertia } from './inertia.js';

// `inertiaEq(I, m)` lets the equation editor replace the spatial-inertia
// formula; by default it is the reference spatialInertia().
export function buildRobot(spec, inertiaEq = spatialInertia) {
  const N = spec.joints.length;
  const S = spec.joints.map((j) => (j.S ? j.S.slice() : screwAxis(j.sHat, j.q, j.h ?? 0)));

  const Mlink = spec.links.map((l) => l.Mlink.map((r) => r.slice()));
  Mlink.push(spec.Mee.map((r) => r.slice())); // index N = tool frame {N+1}

  const Mrel = [];
  for (let i = 0; i <= N; i++) {
    const Mprev = i === 0 ? eye(4) : Mlink[i - 1];
    Mrel.push(matMul(transInv(Mlink[i]), Mprev));
  }

  const A = [];
  const G = [];
  const Gref = []; // always the reference formula, whatever the editor says
  const Irot = [];
  const mass = [];
  for (let i = 0; i < N; i++) {
    A.push(matVec(adjoint(transInv(Mlink[i])), S[i]));
    const I = shapeInertia(spec.links[i].mass, spec.links[i].shape);
    Irot.push(I);
    mass.push(spec.links[i].mass);
    G.push(inertiaEq(I, spec.links[i].mass));
    Gref.push(spatialInertia(I, spec.links[i].mass));
  }

  return {
    name: spec.name,
    N,
    S,
    Mlink,
    Mrel,
    Mee: Mlink[N],
    A,
    G,
    Gref,
    Irot,
    mass,
    gravity: spec.gravity.slice(),
    damping: (spec.damping ?? new Array(N).fill(0)).slice(),
    spec,
  };
}

// Sanity checks on a spec. Returns a list of human-readable problems; the UI
// refuses to apply a spec with problems, so a bad input never reaches the
// simulation.
export function specProblems(spec) {
  const problems = [];
  const isNum = (x) => typeof x === 'number' && Number.isFinite(x);
  const N = spec.joints?.length ?? 0;
  if (N < 1) problems.push('robot needs at least one joint');
  if ((spec.links?.length ?? 0) !== N) problems.push('need exactly one link per joint');
  if (!spec.gravity || spec.gravity.length !== 3 || !spec.gravity.every(isNum)) {
    problems.push('gravity must be a 3-vector of numbers');
  }
  (spec.joints ?? []).forEach((j, i) => {
    if (j.S) {
      if (j.S.length !== 6 || !j.S.every(isNum)) problems.push(`joint ${i + 1}: S must be 6 numbers`);
      else if (norm(j.S.slice(0, 3)) < 1e-9 && norm(j.S.slice(3)) < 1e-9) problems.push(`joint ${i + 1}: S is zero`);
    } else {
      if (!j.sHat || !j.sHat.every(isNum) || norm(j.sHat) < 1e-9) problems.push(`joint ${i + 1}: axis direction ŝ must be non-zero`);
      if (!j.q || !j.q.every(isNum)) problems.push(`joint ${i + 1}: point q must be numbers`);
    }
  });
  (spec.links ?? []).forEach((l, i) => {
    if (!isNum(l.mass) || l.mass <= 0) problems.push(`link ${i + 1}: mass must be positive`);
    if (!isTransform(l.Mlink)) problems.push(`link ${i + 1}: home pose is not a valid transform`);
    try {
      const shape = l.shape;
      for (const [k, v] of Object.entries(shape)) {
        if (k !== 'type' && k !== 'I' && (!isNum(v) || v <= 0)) problems.push(`link ${i + 1}: shape ${k} must be positive`);
      }
      if (isNum(l.mass) && l.mass > 0) {
        for (const p of inertiaProblems(shapeInertia(l.mass, shape))) problems.push(`link ${i + 1}: ${p}`);
      }
    } catch (e) {
      problems.push(`link ${i + 1}: ${e.message}`);
    }
  });
  if (!isTransform(spec.Mee)) problems.push('tool home pose is not a valid transform');
  if (spec.damping && (!spec.damping.every(isNum) || spec.damping.some((b) => b < 0))) {
    problems.push('joint damping must be non-negative numbers');
  }
  return problems;
}

// A 4x4 with an orthonormal, right-handed rotation block and bottom row 0 0 0 1.
export function isTransform(T) {
  if (!Array.isArray(T) || T.length !== 4 || T.some((r) => !Array.isArray(r) || r.length !== 4)) return false;
  for (const row of T) for (const v of row) if (!Number.isFinite(v)) return false;
  if (Math.abs(T[3][0]) + Math.abs(T[3][1]) + Math.abs(T[3][2]) + Math.abs(T[3][3] - 1) > 1e-9) return false;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += T[k][i] * T[k][j];
      if (Math.abs(s - (i === j ? 1 : 0)) > 1e-6) return false;
    }
  }
  const d =
    T[0][0] * (T[1][1] * T[2][2] - T[1][2] * T[2][1]) -
    T[0][1] * (T[1][0] * T[2][2] - T[1][2] * T[2][0]) +
    T[0][2] * (T[1][0] * T[2][1] - T[1][1] * T[2][0]);
  return d > 0;
}

// Deep copy of a spec (specs are plain JSON).
export function cloneSpec(spec) {
  return JSON.parse(JSON.stringify(spec));
}

// The numbers an independent implementation needs, with shapes resolved to
// inertia matrices. Used by tools/export_robot.mjs for the Python reference.
export function referenceSpec(spec) {
  const robot = buildRobot(spec);
  return {
    name: spec.name,
    gravity: spec.gravity,
    S: robot.S,
    Mlink: robot.Mlink.slice(0, robot.N),
    Mee: robot.Mee,
    mass: robot.mass,
    inertia: robot.Irot,
  };
}
