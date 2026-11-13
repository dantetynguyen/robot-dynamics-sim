// linalg.js — small dense linear algebra on plain arrays.
//
// Matrices are arrays of rows: A[row][col]. Vectors are plain arrays.
// Everything returns new arrays (no in-place mutation), which keeps the
// physics code easy to read: `matVec(G, Vdot)` reads like G·V̇.
//
// These are deliberately simple loops. The matrices in this app are at most
// 6x6, so clarity beats cleverness.

// ---------- constructors ----------

export function zeros(rows, cols) {
  const A = new Array(rows);
  for (let i = 0; i < rows; i++) A[i] = new Array(cols).fill(0);
  return A;
}

export function zerosVec(n) {
  return new Array(n).fill(0);
}

export function eye(n) {
  const A = zeros(n, n);
  for (let i = 0; i < n; i++) A[i][i] = 1;
  return A;
}

export function diag(values) {
  const A = zeros(values.length, values.length);
  for (let i = 0; i < values.length; i++) A[i][i] = values[i];
  return A;
}

export function copyMat(A) {
  return A.map((row) => row.slice());
}

// ---------- vectors ----------

export function vecAdd(a, b) {
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + b[i];
  return out;
}

export function vecSub(a, b) {
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] - b[i];
  return out;
}

export function vecScale(a, s) {
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] * s;
  return out;
}

export function dot(a, b) {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
}

export function norm(a) {
  return Math.sqrt(dot(a, a));
}

export function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function normalize(a) {
  const n = norm(a);
  return n === 0 ? a.slice() : vecScale(a, 1 / n);
}

export function maxAbs(a) {
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i]));
  return m;
}

// ---------- matrices ----------

export function matMul(A, B) {
  const n = A.length, m = B[0].length, p = B.length;
  const C = new Array(n);
  for (let i = 0; i < n; i++) {
    const Ai = A[i];
    const Ci = new Array(m);
    for (let j = 0; j < m; j++) {
      let s = 0;
      for (let k = 0; k < p; k++) s += Ai[k] * B[k][j];
      Ci[j] = s;
    }
    C[i] = Ci;
  }
  return C;
}

export function matVec(A, x) {
  const n = A.length, m = x.length;
  const y = new Array(n);
  for (let i = 0; i < n; i++) {
    const Ai = A[i];
    let s = 0;
    for (let j = 0; j < m; j++) s += Ai[j] * x[j];
    y[i] = s;
  }
  return y;
}

// Aᵀ x without forming Aᵀ.
export function matTVec(A, x) {
  const n = A.length, m = A[0].length;
  const y = new Array(m).fill(0);
  for (let i = 0; i < n; i++) {
    const Ai = A[i], xi = x[i];
    for (let j = 0; j < m; j++) y[j] += Ai[j] * xi;
  }
  return y;
}

export function transpose(A) {
  const n = A.length, m = A[0].length;
  const T = new Array(m);
  for (let j = 0; j < m; j++) {
    const Tj = new Array(n);
    for (let i = 0; i < n; i++) Tj[i] = A[i][j];
    T[j] = Tj;
  }
  return T;
}

export function matAdd(A, B) {
  return A.map((row, i) => row.map((v, j) => v + B[i][j]));
}

export function matSub(A, B) {
  return A.map((row, i) => row.map((v, j) => v - B[i][j]));
}

export function matScale(A, s) {
  return A.map((row) => row.map((v) => v * s));
}

export function maxAbsMat(A) {
  let m = 0;
  for (const row of A) for (const v of row) m = Math.max(m, Math.abs(v));
  return m;
}

export function column(A, j) {
  return A.map((row) => row[j]);
}

// Build a matrix from a list of column vectors.
export function fromColumns(cols) {
  const n = cols[0].length, m = cols.length;
  const A = zeros(n, m);
  for (let j = 0; j < m; j++) for (let i = 0; i < n; i++) A[i][j] = cols[j][i];
  return A;
}

// ---------- solving ----------

// Solve A x = b by Gaussian elimination with partial pivoting.
// Returns null if A is numerically singular.
export function solve(A, b) {
  const n = A.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(M[r][col]) > Math.abs(M[pivot][col])) pivot = r;
    }
    if (Math.abs(M[pivot][col]) < 1e-14) return null;
    if (pivot !== col) { const t = M[pivot]; M[pivot] = M[col]; M[col] = t; }
    for (let r = col + 1; r < n; r++) {
      const f = M[r][col] / M[col][col];
      if (f === 0) continue;
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  const x = new Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let s = M[i][n];
    for (let j = i + 1; j < n; j++) s -= M[i][j] * x[j];
    x[i] = s / M[i][i];
  }
  return x;
}

export function inv(A) {
  const n = A.length;
  const cols = [];
  for (let j = 0; j < n; j++) {
    const e = zerosVec(n);
    e[j] = 1;
    const x = solve(A, e);
    if (x === null) return null;
    cols.push(x);
  }
  return fromColumns(cols);
}

export function det(A) {
  const n = A.length;
  const M = copyMat(A);
  let d = 1;
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(M[r][col]) > Math.abs(M[pivot][col])) pivot = r;
    }
    if (M[pivot][col] === 0) return 0;
    if (pivot !== col) { const t = M[pivot]; M[pivot] = M[col]; M[col] = t; d = -d; }
    d *= M[col][col];
    for (let r = col + 1; r < n; r++) {
      const f = M[r][col] / M[col][col];
      for (let c = col; c < n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return d;
}

// ---------- symmetric eigen-decomposition (cyclic Jacobi) ----------
//
// For a symmetric matrix A returns { values, vectors } with A v_k = λ_k v_k,
// eigenvalues sorted descending, vectors[k] the k-th unit eigenvector.
// Used for the ellipsoids (eigen-structure of J Jᵀ), the positive-definite
// check on M(θ), and the null space of Jᵀ.
export function eigSym(Ain) {
  const n = Ain.length;
  const A = copyMat(Ain);
  const V = eye(n);
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(A[p][q]) < 1e-300) continue;
        const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = A[k][p], akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = A[p][k], aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const order = [...Array(n).keys()].sort((i, j) => A[j][j] - A[i][i]);
  return {
    values: order.map((i) => A[i][i]),
    vectors: order.map((i) => column(V, i)),
  };
}

// True if a symmetric matrix is positive definite (all eigenvalues > tol).
export function isPositiveDefinite(A, tol = 1e-12) {
  return eigSym(A).values.every((v) => v > tol);
}

// Largest |A - Aᵀ| entry: zero for a symmetric matrix.
export function asymmetry(A) {
  let m = 0;
  for (let i = 0; i < A.length; i++) {
    for (let j = i + 1; j < A.length; j++) m = Math.max(m, Math.abs(A[i][j] - A[j][i]));
  }
  return m;
}

export function allFinite(a) {
  for (let i = 0; i < a.length; i++) {
    const v = a[i];
    if (Array.isArray(v)) { if (!allFinite(v)) return false; }
    else if (!Number.isFinite(v)) return false;
  }
  return true;
}
