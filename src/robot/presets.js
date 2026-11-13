// presets.js — robot descriptions that ship with the app.
//
// To add a robot: write another function that returns a spec (see model.js
// for the format) and add it to PRESETS. Nothing else needs to change.

import { rotY, rpToTrans } from '../math/lie.js';
import { eye } from '../math/linalg.js';

// A generic tabletop RRR arm: waist yaw, shoulder pitch, elbow pitch.
// The geometry and masses are this project's own round-ish numbers, chosen to
// feel like a small collaborative arm. It is not any assigned or textbook robot.
//
// Home configuration: arm stretched out horizontally along +x.
//
//   z                 joint 2 (shoulder)      joint 3 (elbow)        tool
//   |                   o======= link 2 =======o===== link 3 =====>  *
//   |  link 1           |
//   |  (column)         |  d1
//   o-- joint 1 (waist, about z) ------------------------------------> x
//
// Positive θ2 and θ3 lift the arm (their axes point along −y).
export function rrrCobot() {
  const d1 = 0.32;   // shoulder height above the base, m
  const a2 = 0.38;   // upper-arm length (shoulder axis to elbow axis), m
  const a3 = 0.33;   // forearm length (elbow axis to tool point), m
  const y2 = 0.07;   // sideways offset of the upper arm's centre line, m
  const y3 = 0.02;   // sideways offset of the forearm's centre line, m

  // Link CoM frames {2} and {3} are rotated so their local z axis runs along
  // the link (space +x at home). That matches the cylinder inertia formula
  // (axis along local z) and keeps the code honest about frame changes.
  const alongX = rotY(Math.PI / 2);

  return {
    name: 'RRR tabletop arm',
    gravity: [0, 0, -9.81],
    joints: [
      { sHat: [0, 0, 1], q: [0, 0, 0], h: 0 },
      { sHat: [0, -1, 0], q: [0, 0, d1], h: 0 },
      { sHat: [0, -1, 0], q: [a2, 0, d1], h: 0 },
    ],
    links: [
      {
        mass: 3.4,
        Mlink: rpToTrans(eye(3), [0, 0, 0.18]),
        shape: { type: 'cylinder', r: 0.055, h: 0.26 },
      },
      {
        mass: 2.3,
        Mlink: rpToTrans(alongX, [a2 / 2, y2, d1]),
        shape: { type: 'cylinder', r: 0.038, h: a2 },
      },
      {
        mass: 1.3,
        Mlink: rpToTrans(alongX, [a2 + 0.15, y3, d1]),
        shape: { type: 'cylinder', r: 0.03, h: a3 },
      },
    ],
    Mee: rpToTrans(eye(3), [a2 + a3, y3, d1]),
    damping: [0, 0, 0],
  };
}

export const PRESETS = {
  'rrr-cobot': rrrCobot,
};

export const DEFAULT_PRESET = 'rrr-cobot';
