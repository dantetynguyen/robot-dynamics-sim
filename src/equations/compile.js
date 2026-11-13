// compile.js — turns a syntax tree from parser.js into a JavaScript closure
// that evaluates it.
//
// Safety: the tree is walked by this file only. Every name must be either a
// variable the app explicitly offers for that equation or a function in the
// FUNCTIONS whitelist below; anything else is rejected at compile time, before
// the equation ever runs. There is no eval, no `new Function`, and no way to
// reach JavaScript objects from an expression.
//
// Values are numbers, vectors (plain arrays, treated as columns) or matrices
// (arrays of rows). An n×1 result is always returned as a vector and a 1×1
// result as a number, so `F'*A` gives a plain number.

import {
  cross, det, diag, dot, eye, inv, matMul, matTVec, matVec, norm, transpose, zeros, zerosVec,
} from '../math/linalg.js';
import { ad, adjoint, matrixExp6, skew, transInv } from '../math/lie.js';
import { EquationError, parse } from './parser.js';

// ---------- value helpers ----------

const isNum = (x) => typeof x === 'number';
const isVec = (x) => Array.isArray(x) && !Array.isArray(x[0]);
const isMat = (x) => Array.isArray(x) && Array.isArray(x[0]);

export function describe(x) {
  if (isNum(x)) return 'a number';
  if (isVec(x)) return `a ${x.length}-vector`;
  if (isMat(x)) return `a ${x.length}×${x[0].length} matrix`;
  return 'something unexpected';
}

// n×1 matrix -> vector, 1×1 -> number.
function canon(x) {
  if (isMat(x)) {
    if (x[0].length === 1) x = x.map((row) => row[0]);
    else return x;
  }
  if (isVec(x) && x.length === 1) return x[0];
  return x;
}

const asMat = (x) => (isNum(x) ? [[x]] : isVec(x) ? x.map((v) => [v]) : x);

function fail(message, node) {
  throw new EquationError(message, node ? node.pos : null);
}

function elementwise(a, b, f, what, node) {
  if (isNum(a) && isNum(b)) return f(a, b);
  if (isNum(a)) return isVec(b) ? b.map((v) => f(a, v)) : b.map((r) => r.map((v) => f(a, v)));
  if (isNum(b)) return isVec(a) ? a.map((v) => f(v, b)) : a.map((r) => r.map((v) => f(v, b)));
  if (isVec(a) && isVec(b) && a.length === b.length) return a.map((v, i) => f(v, b[i]));
  if (isMat(a) && isMat(b) && a.length === b.length && a[0].length === b[0].length) {
    return a.map((r, i) => r.map((v, j) => f(v, b[i][j])));
  }
  fail(`cannot ${what} ${describe(a)} and ${describe(b)}: sizes do not match`, node);
}

function multiply(a, b, node) {
  if (isNum(a) || isNum(b)) return elementwise(a, b, (x, y) => x * y, 'multiply', node);
  if (isMat(a) && isVec(b)) {
    if (a[0].length !== b.length) fail(`cannot multiply ${describe(a)} by ${describe(b)}`, node);
    return canon(matVec(a, b));
  }
  if (isMat(a) && isMat(b)) {
    if (a[0].length !== b.length) fail(`cannot multiply ${describe(a)} by ${describe(b)}`, node);
    return canon(matMul(a, b));
  }
  if (isVec(a) && isMat(b)) {
    if (b.length !== 1) fail(`cannot multiply ${describe(a)} by ${describe(b)}`, node);
    return a.map((v) => b[0].map((w) => v * w)); // column × row = outer product
  }
  fail("cannot multiply two column vectors; use a'*b for the dot product or a.*b element-wise", node);
}

function power(a, b, node) {
  if (isNum(a) && isNum(b)) return Math.pow(a, b);
  if (isMat(a) && isNum(b) && a.length === a[0].length && Number.isInteger(b) && b >= 0 && b <= 16) {
    let out = eye(a.length);
    for (let k = 0; k < b; k++) out = matMul(out, a);
    return out;
  }
  fail(`cannot raise ${describe(a)} to the power ${describe(b)}; use .^ for element-wise powers`, node);
}

function transposeValue(x) {
  if (isNum(x)) return x;
  if (isVec(x)) return [x.slice()];        // column -> 1×n row
  return canon(transpose(x));              // 1×n row -> vector
}

// [a, b; c, d] with blocks: entries in a row must have equal heights, rows
// must have equal widths.
function concatenate(rows, node) {
  const built = rows.map((entries) => {
    const blocks = entries.map(asMat);
    const h = blocks[0].length;
    if (blocks.some((b) => b.length !== h)) fail('matrix row has entries of different heights', node);
    return Array.from({ length: h }, (_, r) => blocks.flatMap((b) => b[r]));
  });
  const w = built[0][0].length;
  if (built.some((block) => block[0].length !== w)) fail('matrix rows have different widths', node);
  return canon(built.flat());
}

// ---------- whitelisted functions ----------

function need(cond, message) {
  if (!cond) throw new EquationError(message);
}
const size = (n, name) => {
  need(isNum(n) && Number.isInteger(n) && n >= 1 && n <= 64, `${name}: size must be a whole number from 1 to 64`);
  return n;
};
const vecN = (x, n, name) => { need(isVec(x) && x.length === n, `${name} expects a ${n}-vector, got ${describe(x)}`); return x; };
const square = (x, name) => { need(isMat(x) && x.length === x[0].length, `${name} expects a square matrix, got ${describe(x)}`); return x; };
const mapScalar = (f) => (x) => (isNum(x) ? f(x) : isVec(x) ? x.map(f) : x.map((r) => r.map(f)));

// name -> { args: [min, max], fn }. Object.create(null) so that names like
// "constructor" or "__proto__" can never resolve to anything.
export const FUNCTIONS = Object.assign(Object.create(null), {
  // element-wise scalar functions
  sin: { args: [1, 1], fn: mapScalar(Math.sin) },
  cos: { args: [1, 1], fn: mapScalar(Math.cos) },
  tan: { args: [1, 1], fn: mapScalar(Math.tan) },
  tanh: { args: [1, 1], fn: mapScalar(Math.tanh) },
  exp: { args: [1, 1], fn: mapScalar(Math.exp) },
  log: { args: [1, 1], fn: mapScalar(Math.log) },
  sqrt: { args: [1, 1], fn: mapScalar(Math.sqrt) },
  abs: { args: [1, 1], fn: mapScalar(Math.abs) },
  sign: { args: [1, 1], fn: mapScalar(Math.sign) },
  min: { args: [2, 2], fn: (a, b) => elementwise(a, b, Math.min, 'take the minimum of') },
  max: { args: [2, 2], fn: (a, b) => elementwise(a, b, Math.max, 'take the maximum of') },
  // saturate x to ±limit (handy for torque limits in a control law)
  sat: { args: [2, 2], fn: (x, lim) => elementwise(x, lim, (v, l) => Math.max(-l, Math.min(l, v)), 'saturate') },

  // constructors
  eye: { args: [1, 1], fn: (n) => canon(eye(size(n, 'eye'))) },
  zeros: { args: [1, 2], fn: (n, m = 1) => (m === 1 ? canon(zerosVec(size(n, 'zeros'))) : canon(zeros(size(n, 'zeros'), size(m, 'zeros')))) },
  ones: { args: [1, 2], fn: (n, m = 1) => (m === 1 ? canon(new Array(size(n, 'ones')).fill(1)) : canon(Array.from({ length: size(n, 'ones') }, () => new Array(size(m, 'ones')).fill(1)))) },
  diag: { args: [1, 1], fn: (v) => { need(isVec(v), `diag expects a vector, got ${describe(v)}`); return diag(v); } },
  blkdiag: {
    args: [2, 4],
    fn: (...blocks) => {
      const mats = blocks.map(asMat);
      const n = mats.reduce((a, b) => a + b.length, 0), m = mats.reduce((a, b) => a + b[0].length, 0);
      const out = zeros(n, m);
      let r0 = 0, c0 = 0;
      for (const b of mats) {
        for (let i = 0; i < b.length; i++) for (let j = 0; j < b[0].length; j++) out[r0 + i][c0 + j] = b[i][j];
        r0 += b.length; c0 += b[0].length;
      }
      return out;
    },
  },

  // general linear algebra
  transpose: { args: [1, 1], fn: transposeValue },
  inv: { args: [1, 1], fn: (A) => { if (isNum(A)) return 1 / A; const out = inv(square(A, 'inv')); need(out !== null, 'inv: matrix is singular'); return out; } },
  det: { args: [1, 1], fn: (A) => det(square(A, 'det')) },
  trace: { args: [1, 1], fn: (A) => square(A, 'trace').reduce((s, r, i) => s + r[i], 0) },
  norm: { args: [1, 1], fn: (v) => { need(isVec(v) || isNum(v), `norm expects a vector, got ${describe(v)}`); return isNum(v) ? Math.abs(v) : norm(v); } },
  dot: { args: [2, 2], fn: (a, b) => { need(isVec(a) && isVec(b) && a.length === b.length, 'dot expects two vectors of the same length'); return dot(a, b); } },
  cross: { args: [2, 2], fn: (a, b) => cross(vecN(a, 3, 'cross'), vecN(b, 3, 'cross')) },

  // screw theory (same operations as src/math/lie.js)
  skew: { args: [1, 1], fn: (w) => skew(vecN(w, 3, 'skew')) },                 // [ω]
  ad: { args: [1, 1], fn: (V) => ad(vecN(V, 6, 'ad')) },                       // little adjoint ad_V
  Ad: { args: [1, 1], fn: (T) => { need(isMat(T) && T.length === 4 && T[0].length === 4, `Ad expects a 4×4 transform, got ${describe(T)}`); return cachedAdjoint(T); } },
  TransInv: { args: [1, 1], fn: (T) => { need(isMat(T) && T.length === 4 && T[0].length === 4, 'TransInv expects a 4×4 transform'); return transInv(T); } },
  expm6: { args: [2, 2], fn: (S, th) => { need(isNum(th), 'expm6(S, theta): theta must be a number'); return matrixExp6(vecN(S, 6, 'expm6'), th); } },
  angular: { args: [1, 1], fn: (V) => vecN(V, 6, 'angular').slice(0, 3) },     // top half of a twist/wrench
  linear: { args: [1, 1], fn: (V) => vecN(V, 6, 'linear').slice(3) },          // bottom half
});

const CONSTANTS = Object.assign(Object.create(null), { pi: Math.PI });

// The same transform object is passed to Ad() many times per time step (once
// per RNE call at that configuration), so remember its adjoint. Values are
// never mutated in this codebase, which is what makes this safe.
const adjointCache = new WeakMap();
function cachedAdjoint(T) {
  let A = adjointCache.get(T);
  if (!A) { A = adjoint(T); adjointCache.set(T, A); }
  return A;
}

// ---------- compiler ----------

// compile(text, variableNames) -> { evaluate(scope), uses }
//   variableNames  the names this equation is allowed to read
//   evaluate(scope) the compiled closure; scope is an object with those names
//   uses           Set of variable names the expression actually reads
// Throws EquationError on a syntax error or an unknown name.
export function compile(text, variableNames) {
  const allowed = new Set(variableNames);
  const uses = new Set();
  const tree = parse(text);

  function build(node) {
    switch (node.type) {
      case 'num': {
        const v = node.value;
        return () => v;
      }
      case 'name': {
        const name = node.name;
        if (allowed.has(name)) {
          uses.add(name);
          return (scope) => scope[name];
        }
        if (name in CONSTANTS) { const v = CONSTANTS[name]; return () => v; }
        if (name in FUNCTIONS) fail(`"${name}" is a function; write ${name}(…)`, node);
        fail(`unknown name "${name}". Available here: ${[...allowed].join(', ')}`, node);
        break;
      }
      case 'neg': {
        const x = build(node.x);
        return (scope) => elementwise(-1, x(scope), (a, b) => a * b, 'negate', node);
      }
      case 'transpose': {
        const x = build(node.x);
        return (scope) => transposeValue(x(scope));
      }
      case 'binary': {
        if (node.op === '*') return buildProduct(node);
        const a = build(node.a), b = build(node.b);
        switch (node.op) {
          case '+': return (s) => elementwise(a(s), b(s), (x, y) => x + y, 'add', node);
          case '-': return (s) => elementwise(a(s), b(s), (x, y) => x - y, 'subtract', node);
          case '.*': return (s) => elementwise(a(s), b(s), (x, y) => x * y, 'multiply', node);
          case './': return (s) => elementwise(a(s), b(s), (x, y) => x / y, 'divide', node);
          case '.^': return (s) => elementwise(a(s), b(s), Math.pow, 'take the power of', node);
          case '^': return (s) => power(a(s), b(s), node);
          case '/': return (s) => {
            const d = b(s);
            if (!isNum(d)) fail(`can only divide by a number, not ${describe(d)}; use inv(A)*B or ./`, node);
            return elementwise(a(s), d, (x, y) => x / y, 'divide', node);
          };
          default: fail(`unknown operator ${node.op}`, node);
        }
        break;
      }
      case 'matrix': {
        const rows = node.rows.map((entries) => entries.map(build));
        return (scope) => concatenate(rows.map((entries) => entries.map((e) => e(scope))), node);
      }
      case 'apply': {
        // A name followed by (...) is indexing if it is a variable, otherwise
        // a call to a whitelisted function.
        if (allowed.has(node.name)) return buildIndex(node);
        const f = FUNCTIONS[node.name];
        if (!f) fail(`unknown function "${node.name}"`, node);
        if (node.args.length < f.args[0] || node.args.length > f.args[1]) {
          fail(`${node.name} takes ${f.args[0] === f.args[1] ? f.args[0] : `${f.args[0]} to ${f.args[1]}`} argument(s)`, node);
        }
        const args = node.args.map((a) => {
          if (a.type === 'range' || a.type === 'colon') fail('a range (a:b) can only be used to index a variable', a);
          return build(a);
        });
        const fn = f.fn;
        return (scope) => {
          try {
            return fn(...args.map((a) => a(scope)));
          } catch (e) {
            if (e instanceof EquationError && e.position === null) e.position = node.pos;
            throw e;
          }
        };
      }
      case 'range':
      case 'colon':
        fail('a range (a:b) can only be used to index a variable', node);
        break;
      default:
        fail('unsupported syntax', node);
    }
    return null;
  }

  // A chain of products a*b*…*z. Matrix products are associative, so when the
  // chain ends in a vector (the usual case: ad(V)'*G*V, Ad(T)'*F, G*Vd) it is
  // folded from the right — a few matrix×vector products instead of
  // matrix×matrix ones — and a transposed matrix is applied without building
  // its transpose. Any other mix of sizes falls back to plain left-to-right
  // multiplication, which also produces the error messages.
  function buildProduct(node) {
    const factors = []; // { f: closure, transposed: bool }
    const collect = (n) => {
      if (n.type === 'binary' && n.op === '*') { collect(n.a); collect(n.b); }
      else if (n.type === 'transpose') factors.push({ f: build(n.x), transposed: true });
      else factors.push({ f: build(n), transposed: false });
    };
    // only the left spine is a chain: a*b*c parses as (a*b)*c
    const spine = (n) => {
      if (n.type === 'binary' && n.op === '*') { spine(n.a); collect(n.b); } else collect(n);
    };
    spine(node);
    const count = factors.length;
    return (scope) => {
      const vals = new Array(count);
      for (let i = 0; i < count; i++) vals[i] = factors[i].f(scope);
      const last = vals[count - 1];
      if (isVec(last) && !factors[count - 1].transposed) {
        let v = last, scalar = 1, ok = true;
        for (let i = count - 2; i >= 0 && ok; i--) {
          const x = vals[i];
          if (isNum(x)) scalar *= x;
          else if (isMat(x) && !factors[i].transposed && x[0].length === v.length && x.length > 1) v = matVec(x, v);
          else if (isMat(x) && factors[i].transposed && x.length === v.length && x[0].length > 1) v = matTVec(x, v);
          else if (isVec(x) && factors[i].transposed && x.length === v.length && i === 0) { v = dot(x, v); }
          else ok = false;
          if (ok && isNum(v) && i > 0) ok = false; // a number mid-chain: let the general path handle it
        }
        if (ok) {
          if (isNum(v)) return v * scalar;
          if (scalar !== 1) { const out = new Array(v.length); for (let k = 0; k < v.length; k++) out[k] = v[k] * scalar; return out; }
          return v;
        }
      }
      let acc = factors[0].transposed ? transposeValue(vals[0]) : vals[0];
      for (let i = 1; i < count; i++) acc = multiply(acc, factors[i].transposed ? transposeValue(vals[i]) : vals[i], node);
      return acc;
    };
  }

  // X(i), X(a:b), X(i,j), X(:,j) … with 1-based indices.
  function buildIndex(node) {
    const name = node.name;
    uses.add(name);
    const selectors = node.args.map((a) => {
      if (a.type === 'colon') return () => null; // whole dimension
      if (a.type === 'range') {
        const lo = build(a.a), hi = build(a.b);
        return (scope) => [lo(scope), hi(scope)];
      }
      const one = build(a);
      return (scope) => { const v = one(scope); return [v, v, true]; };
    });
    const pick = (sel, n, scope) => {
      const r = sel(scope);
      if (r === null) return { from: 0, to: n - 1, single: false };
      const [lo, hi, single] = r;
      if (!Number.isInteger(lo) || !Number.isInteger(hi) || lo < 1 || hi > n || lo > hi) {
        fail(`index out of range for "${name}" (valid: 1 to ${n})`, node);
      }
      return { from: lo - 1, to: hi - 1, single: !!single };
    };
    return (scope) => {
      const x = scope[name];
      if (isVec(x) && selectors.length === 1) {
        const r = pick(selectors[0], x.length, scope);
        return r.single ? x[r.from] : canon(x.slice(r.from, r.to + 1));
      }
      if (isMat(x) && selectors.length === 2) {
        const r = pick(selectors[0], x.length, scope), c = pick(selectors[1], x[0].length, scope);
        const out = [];
        for (let i = r.from; i <= r.to; i++) out.push(x[i].slice(c.from, c.to + 1));
        return canon(out);
      }
      fail(`"${name}" is ${describe(x)}; it needs ${isMat(x) ? 'two indices' : 'one index'}`, node);
      return null;
    };
  }

  const root = build(tree);
  return { evaluate: (scope) => canon(root(scope)), uses };
}
