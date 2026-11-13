"""Independent numpy reference for the simulator's dynamics.

This file deliberately does NOT follow the structure of the JavaScript code.
The JavaScript uses the body-frame recursive Newton-Euler algorithm with
closed-form (Rodrigues) exponentials. Here everything is done in the SPACE
frame, without recursion, with scipy's general matrix exponential:

  FK      T_0i = expm([S_1]th_1) ... expm([S_i]th_i) M_i
  J_s     column i = Ad_{expm product before joint i} S_i        (MR Eq. 5.11)
  M       sum_i J_i^T G_i^s J_i, with G_i^s the link inertia moved to {0}
          and J_i = J_s with the columns after joint i zeroed    (MR Eq. 8.57)
  c       projected space-frame Newton-Euler:
          c_j = sum_{i>=j} J_sj . ( G_i^s Jdot_i dth - ad_{V_i}^T G_i^s V_i )
          with d/dt J_sj = ad_{V_j} J_sj
  g       gradient of potential energy written with point Jacobians:
          g_j = - sum_{i>=j} m_i gvec . (w_j x p_ci + v_j)

If these agree with the JavaScript to 1e-9 at random states, two different
derivations in two languages agree, which is the point.

Usage:  python3 reference/generate_fixtures.py
Writes: tests/fixtures/reference.json
"""
import json
import os

import numpy as np
from scipy.linalg import expm

HERE = os.path.dirname(os.path.abspath(__file__))
N_STATES = 100


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], float)


def se3(V):
    out = np.zeros((4, 4))
    out[:3, :3] = skew(V[:3])
    out[:3, 3] = V[3:]
    return out


def Ad(T):
    R, p = T[:3, :3], T[:3, 3]
    out = np.zeros((6, 6))
    out[:3, :3] = R
    out[3:, 3:] = R
    out[3:, :3] = skew(p) @ R
    return out


def ad(V):
    out = np.zeros((6, 6))
    out[:3, :3] = skew(V[:3])
    out[3:, 3:] = skew(V[:3])
    out[3:, :3] = skew(V[3:])
    return out


class Robot:
    def __init__(self, d):
        self.name = d["name"]
        self.S = [np.array(s, float) for s in d["S"]]
        self.Mlink = [np.array(m, float) for m in d["Mlink"]]
        self.Mee = np.array(d["Mee"], float)
        self.mass = [float(m) for m in d["mass"]]
        self.inertia = [np.array(i, float) for i in d["inertia"]]
        self.g = np.array(d["gravity"], float)
        self.n = len(self.S)

    def partial_products(self, th):
        P = [np.eye(4)]
        for i in range(self.n):
            P.append(P[-1] @ expm(se3(self.S[i]) * th[i]))
        return P

    def fk(self, th):
        return self.partial_products(th)[-1] @ self.Mee

    def link_poses(self, th):
        P = self.partial_products(th)
        return [P[i + 1] @ self.Mlink[i] for i in range(self.n)]

    def jac_space(self, th):
        P = self.partial_products(th)
        return np.column_stack([Ad(P[i]) @ self.S[i] for i in range(self.n)])

    def space_inertias(self, th):
        out = []
        for i, T in enumerate(self.link_poses(th)):
            G = np.zeros((6, 6))
            G[:3, :3] = self.inertia[i]
            G[3:, 3:] = self.mass[i] * np.eye(3)
            A = Ad(np.linalg.inv(T))
            out.append(A.T @ G @ A)
        return out

    def mass_matrix(self, th):
        J = self.jac_space(th)
        Gs = self.space_inertias(th)
        M = np.zeros((self.n, self.n))
        for i in range(self.n):
            Ji = J.copy()
            Ji[:, i + 1:] = 0
            M += Ji.T @ Gs[i] @ Ji
        return M

    def coriolis(self, th, dth):
        J = self.jac_space(th)
        Gs = self.space_inertias(th)
        # space twist of each link, and the time derivative of each J column
        V = [J[:, : i + 1] @ dth[: i + 1] for i in range(self.n)]
        Jdot = np.column_stack([ad(V[j]) @ J[:, j] for j in range(self.n)])
        c = np.zeros(self.n)
        for i in range(self.n):
            acc = Jdot[:, : i + 1] @ dth[: i + 1]          # space acceleration, ddth = 0
            wrench = Gs[i] @ acc - ad(V[i]).T @ Gs[i] @ V[i]
            for j in range(i + 1):
                c[j] += J[:, j] @ wrench
        return c

    def gravity(self, th):
        J = self.jac_space(th)
        poses = self.link_poses(th)
        gq = np.zeros(self.n)
        for i in range(self.n):
            pc = poses[i][:3, 3]
            for j in range(i + 1):
                w, v = J[:3, j], J[3:, j]
                gq[j] -= self.mass[i] * self.g @ (np.cross(w, pc) + v)
        return gq


def random_rotation(rng):
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] = -q[:, 0]
    return q


def random_spec(rng):
    """A random 3R arm in the JavaScript spec format (see src/robot/model.js):
    arbitrary axes, arbitrarily rotated CoM frames, full (non-diagonal) inertias."""
    joints, links = [], []
    for _ in range(3):
        s = rng.normal(size=3)
        s /= np.linalg.norm(s)
        joints.append({"sHat": s.tolist(), "q": rng.uniform(-0.4, 0.4, 3).tolist(), "h": 0})
        T = np.eye(4)
        T[:3, :3] = random_rotation(rng)
        T[:3, 3] = rng.uniform(-0.5, 0.5, 3)
        m = float(rng.uniform(0.5, 4.0))
        # principal moments that satisfy the triangle inequality, in a random basis
        a, b, c = rng.uniform(0.05, 0.3, 3)
        principal = m / 5 * np.array([b * b + c * c, a * a + c * c, a * a + b * b])
        Rb = random_rotation(rng)
        I = Rb @ np.diag(principal) @ Rb.T
        I = (I + I.T) / 2
        links.append({"mass": m, "Mlink": T.tolist(), "shape": {"type": "matrix", "I": I.tolist()}})
    Mee = np.eye(4)
    Mee[:3, :3] = random_rotation(rng)
    Mee[:3, 3] = rng.uniform(-0.6, 0.6, 3)
    g = rng.normal(size=3)
    g *= 9.81 / np.linalg.norm(g)
    return {"name": "random 3R (test only)", "gravity": g.tolist(), "joints": joints,
            "links": links, "Mee": Mee.tolist(), "damping": [0, 0, 0]}


def spec_to_reference(spec):
    S = []
    for j in spec["joints"]:
        s, q = np.array(j["sHat"]), np.array(j["q"])
        S.append(np.concatenate([s, -np.cross(s, q)]).tolist())
    return {"name": spec["name"], "gravity": spec["gravity"], "S": S,
            "Mlink": [l["Mlink"] for l in spec["links"]], "Mee": spec["Mee"],
            "mass": [l["mass"] for l in spec["links"]],
            "inertia": [l["shape"]["I"] for l in spec["links"]]}


def states_for(robot, rng):
    out = []
    for _ in range(N_STATES):
        th = rng.uniform(-np.pi, np.pi, robot.n)
        dth = rng.uniform(-3, 3, robot.n)
        out.append({
            "theta": th.tolist(), "dtheta": dth.tolist(),
            "T": robot.fk(th).tolist(),
            "Js": robot.jac_space(th).tolist(),
            "M": robot.mass_matrix(th).tolist(),
            "c": robot.coriolis(th, dth).tolist(),
            "g": robot.gravity(th).tolist(),
        })
    return out


def main():
    rng = np.random.default_rng(501)
    with open(os.path.join(HERE, "robot_default.json")) as f:
        default = json.load(f)
    spec = random_spec(rng)
    fixtures = {
        "about": "Generated by reference/generate_fixtures.py (numpy/scipy, space-frame, non-recursive).",
        "numpy": np.__version__,
        "robots": [
            {"source": "preset", "preset": "rrr-cobot", "reference": default,
             "states": states_for(Robot(default), rng)},
            {"source": "random", "spec": spec,
             "states": states_for(Robot(spec_to_reference(spec)), rng)},
        ],
    }
    out = os.path.join(HERE, "..", "tests", "fixtures", "reference.json")
    with open(out, "w") as f:
        json.dump(fixtures, f)
    print("wrote", os.path.normpath(out), "with", N_STATES, "states per robot")


if __name__ == "__main__":
    main()
