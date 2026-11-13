// trajectory.js — reference trajectories θ_d(t), θ̇_d(t), θ̈_d(t).
//
// Point-to-point moves follow a straight line in joint space,
//   θ(s) = θ_start + s (θ_end − θ_start),  s ∈ [0, 1]            (MR Eq. 9.3)
// driven by a time scaling s(t) that starts and ends at rest (MR §9.2.2.1):
//   cubic    s = 3u² − 2u³            (u = t/T)   zero end velocities
//   quintic  s = 10u³ − 15u⁴ + 6u⁵                zero end velocities and accelerations
// The two s(t) formulas are editable equations ('traj.cubic', 'traj.quintic').
//
// TO ADD A TRAJECTORY TYPE: add an entry to TRAJECTORIES with a `make(cfg, ctx)`
// that returns a function t -> { theta, dtheta, ddtheta }.

// s, ṡ, s̈ for the built-in scalings (exact derivatives).
const EXACT = {
  cubic: (t, T) => { const u = t / T; return [3 * u * u - 2 * u ** 3, (6 * u - 6 * u * u) / T, (6 - 12 * u) / (T * T)]; },
  quintic: (t, T) => {
    const u = t / T;
    return [10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5, (30 * u * u - 60 * u ** 3 + 30 * u ** 4) / T, (60 * u - 180 * u * u + 120 * u ** 3) / (T * T)];
  },
};

// Returns (t, T) -> [s, ṡ, s̈]. If the equation has been edited, its
// derivatives are taken numerically (central differences).
function timeScaling(kind, shared) {
  const id = `traj.${kind}`;
  if (!shared || !shared.isEdited(id)) return EXACT[kind];
  const s = shared.fn(id);
  return (t, T) => {
    const h = 1e-4 * T;
    const s0 = s({ t, T }), sp = s({ t: t + h, T }), sm = s({ t: t - h, T });
    return [s0, (sp - sm) / (2 * h), (sp - 2 * s0 + sm) / (h * h)];
  };
}

function pointToPoint(kind) {
  return (cfg, ctx) => {
    const scaling = timeScaling(kind, ctx.shared);
    const N = cfg.start.length;
    const T = Math.max(cfg.T, 1e-3);
    const dwell = Math.max(cfg.dwell ?? 0.5, 0);
    const delta = cfg.end.map((e, i) => e - cfg.start[i]);
    const at = (s, sd, sdd) => ({
      theta: cfg.start.map((a, i) => a + s * delta[i]),
      dtheta: delta.map((d) => sd * d),
      ddtheta: delta.map((d) => sdd * d),
    });
    const rest = (s) => at(s, 0, 0);
    return (t) => {
      if (t <= 0) return rest(0);
      if (!cfg.repeat) {
        if (t >= T) return rest(1);
        return at(...scaling(t, T));
      }
      // repeat: out, dwell, back, dwell, …
      const period = 2 * (T + dwell);
      const tc = t % period;
      if (tc < T) return at(...scaling(tc, T));
      if (tc < T + dwell) return rest(1);
      if (tc < 2 * T + dwell) { const [s, sd, sdd] = scaling(tc - T - dwell, T); return at(1 - s, -sd, -sdd); }
      return rest(0);
    };
  };
}

export const TRAJECTORIES = {
  hold: {
    name: 'Hold a pose',
    make: (cfg) => {
      const N = cfg.start.length;
      const still = { theta: cfg.start.slice(), dtheta: new Array(N).fill(0), ddtheta: new Array(N).fill(0) };
      return () => still;
    },
  },
  cubic: { name: 'Cubic point-to-point', make: pointToPoint('cubic') },
  quintic: { name: 'Quintic point-to-point', make: pointToPoint('quintic') },
  sine: {
    name: 'Repeating sinusoid',
    // θ_d = centre + amplitude · sin(2π f t)
    make: (cfg) => {
      const w = 2 * Math.PI * cfg.freq;
      return (t) => {
        const s = Math.sin(w * t), c = Math.cos(w * t);
        return {
          theta: cfg.center.map((c0, i) => c0 + cfg.amplitude[i] * s),
          dtheta: cfg.amplitude.map((a) => a * w * c),
          ddtheta: cfg.amplitude.map((a) => -a * w * w * s),
        };
      };
    },
  },
};

// cfg: { type, start, end, T, repeat, dwell, center, amplitude, freq }
// ctx: { shared } — the shared EquationSet (for edited time scalings)
export function makeTrajectory(cfg, ctx = {}) {
  const entry = TRAJECTORIES[cfg.type] ?? TRAJECTORIES.hold;
  return entry.make(cfg, ctx);
}

// Length of one pass of the trajectory, for drawing the desired path.
export function trajectorySpan(cfg) {
  if (cfg.type === 'sine') return 1 / Math.max(cfg.freq, 1e-3);
  if (cfg.type === 'hold') return 0;
  return cfg.repeat ? 2 * (cfg.T + (cfg.dwell ?? 0.5)) : cfg.T;
}
