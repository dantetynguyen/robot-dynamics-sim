// helpers.js — shared test utilities.
//
// `record()` both asserts a numeric acceptance check and writes the measured
// number to tests/results/<file>.json, so `npm run acceptance` can print a
// table of real numbers (see tools/acceptance_report.mjs).

import assert from 'node:assert/strict';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { basename } from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildRobot } from '../src/robot/model.js';
import { PRESETS } from '../src/robot/presets.js';

const resultsDir = fileURLToPath(new URL('./results/', import.meta.url));
const results = [];
let resultsFile = null;

export function useResultsFile(importMetaUrl) {
  resultsFile = resultsDir + basename(fileURLToPath(importMetaUrl)).replace('.test.js', '.json');
}

// Assert value <= limit (or >= limit with dir '>=') and log it.
export function record(name, value, limit, { dir = '<=', unit = '', note = '' } = {}) {
  const pass = dir === '<=' ? value <= limit : value >= limit;
  results.push({ name, value, limit, dir, unit, note, pass });
  if (resultsFile) {
    mkdirSync(resultsDir, { recursive: true });
    writeFileSync(resultsFile, JSON.stringify(results, null, 1));
  }
  assert.ok(pass, `${name}: measured ${value} ${unit}, required ${dir} ${limit}`);
}

// Small deterministic random generator (mulberry32) so tests are repeatable.
export function rng(seed) {
  let a = seed >>> 0;
  const next = () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  next.uniform = (lo, hi) => lo + (hi - lo) * next();
  next.vec = (n, lo, hi) => Array.from({ length: n }, () => next.uniform(lo, hi));
  return next;
}

export function loadFixtures() {
  const path = fileURLToPath(new URL('./fixtures/reference.json', import.meta.url));
  const fx = JSON.parse(readFileSync(path, 'utf8'));
  // Build each robot with THIS repo's JavaScript from the same description
  // the Python reference used.
  return fx.robots.map((r) => ({
    label: r.source === 'preset' ? `default arm (${r.preset})` : 'random 3R arm',
    robot: buildRobot(r.source === 'preset' ? PRESETS[r.preset]() : r.spec),
    states: r.states,
  }));
}

export function maxDiff(a, b) {
  let m = 0;
  const walk = (x, y) => {
    if (Array.isArray(x)) x.forEach((xi, i) => walk(xi, y[i]));
    else m = Math.max(m, Math.abs(x - y));
  };
  walk(a, b);
  return m;
}

export function defaultRobot() {
  return buildRobot(PRESETS['rrr-cobot']());
}
