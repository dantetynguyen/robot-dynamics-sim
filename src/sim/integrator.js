// integrator.js — fixed-step time integration of ẋ = f(t, x).
//
// The notes introduce first-order Euler (Topic 13 pp. 7–8) and name
// fourth-order Runge–Kutta as the usual better choice (Topic 13 p. 8;
// MR §8.5). RK4 samples the slope four times per step and its error shrinks
// like dt⁴; Euler's shrinks like dt and visibly gains energy on a swinging arm.

function axpy(x, k, a) {
  const out = new Array(x.length);
  for (let i = 0; i < x.length; i++) out[i] = x[i] + a * k[i];
  return out;
}

// One classical RK4 step.
export function rk4Step(f, t, x, dt) {
  const k1 = f(t, x);
  const k2 = f(t + dt / 2, axpy(x, k1, dt / 2));
  const k3 = f(t + dt / 2, axpy(x, k2, dt / 2));
  const k4 = f(t + dt, axpy(x, k3, dt));
  const out = new Array(x.length);
  for (let i = 0; i < x.length; i++) out[i] = x[i] + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  return out;
}

// One explicit Euler step (kept for comparison experiments and tests).
export function eulerStep(f, t, x, dt) {
  return axpy(x, f(t, x), dt);
}
