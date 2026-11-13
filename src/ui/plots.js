// plots.js — small live line charts on a 2D canvas (no chart library).
//
// One chart = one quantity on one y-axis, several series, a legend above and
// a crosshair + tooltip on hover. Series colours are CSS variables so they
// follow the light/dark theme.

import { el } from './dom.js';
import { fmt } from './format.js';

function niceStep(span, target) {
  const raw = span / Math.max(target, 1);
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / pow;
  return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * pow;
}

export class StripChart {
  // opts: { title, unit, series: [{ key, label, color: '--s1', dashed }], controls: Node }
  constructor(opts) {
    this.opts = opts;
    this.series = opts.series;
    this.canvas = el('canvas', { class: 'chart-canvas' });
    this.tooltip = el('div', { class: 'chart-tip', hidden: true });
    this.legend = el('div', { class: 'chart-legend' });
    this.titleEl = el('span', { class: 'chart-title' }, opts.title);
    this.el = el('figure', { class: 'chart' },
      el('figcaption', {}, this.titleEl, opts.controls ?? null, this.legend),
      el('div', { class: 'chart-plot' }, this.canvas, this.tooltip));
    this.hoverX = null;
    this.canvas.addEventListener('pointermove', (e) => { this.hoverX = e.offsetX; this.draw(); });
    this.canvas.addEventListener('pointerleave', () => { this.hoverX = null; this.tooltip.hidden = true; this.draw(); });
    this.setSeries(opts.series, opts.title, opts.unit);
  }

  setSeries(series, title = this.opts.title, unit = this.opts.unit) {
    this.series = series;
    this.unit = unit;
    this.titleEl.textContent = unit ? `${title} (${unit})` : title;
    this.legend.replaceChildren(...series.map((s) => el('span', { class: 'legend-item' },
      el('i', { class: `swatch${s.dashed ? ' dashed' : ''}`, style: { '--c': `var(${s.color})` } }), s.label)));
  }

  // times: array of seconds; columns: { key: array of numbers }; window: seconds shown
  setData(times, columns, window) {
    this.times = times;
    this.columns = columns;
    this.window = window;
    this.draw();
  }

  draw() {
    const canvas = this.canvas, dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(this.el);
    const ink = (name) => css.getPropertyValue(name).trim();
    const times = this.times ?? [];
    const padL = 54, padR = 8, padT = 6, padB = 18;
    const pw = w - padL - padR, ph = h - padT - padB;
    ctx.font = '11px "IBM Plex Sans", system-ui, sans-serif';
    if (times.length < 2) {
      ctx.fillStyle = ink('--muted');
      ctx.fillText('Press Run to record data', padL, padT + ph / 2);
      return;
    }
    const t1 = times[times.length - 1], t0 = Math.max(times[0], t1 - this.window);
    let i0 = 0;
    while (i0 < times.length - 1 && times[i0] < t0) i0++;

    // y-range over the visible window, padded, never zero-height
    let lo = Infinity, hi = -Infinity;
    for (const s of this.series) {
      const col = this.columns[s.key];
      if (!col) continue;
      for (let i = i0; i < col.length; i++) { const v = col[i]; if (Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }
    }
    if (!Number.isFinite(lo)) { lo = -1; hi = 1; }
    if (hi - lo < 1e-6) { const mid = (hi + lo) / 2, m = Math.max(Math.abs(mid) * 0.05, 5e-4); lo = mid - m; hi = mid + m; } // do not zoom into rounding noise
    const pad = (hi - lo) * 0.08;
    lo -= pad; hi += pad;
    const X = (t) => padL + ((t - t0) / Math.max(t1 - t0, 1e-9)) * pw;
    const Yp = (v) => padT + (1 - (v - lo) / (hi - lo)) * ph;

    // recessive grid + tick labels
    ctx.lineWidth = 1;
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'right';
    const ys = niceStep(hi - lo, 4);
    for (let v = Math.ceil(lo / ys) * ys; v <= hi; v += ys) {
      const y = Math.round(Yp(v)) + 0.5;
      ctx.strokeStyle = Math.abs(v) < ys * 1e-6 ? ink('--rule-strong') : ink('--rule');
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y); ctx.stroke();
      ctx.fillStyle = ink('--muted');
      ctx.fillText(fmt(Math.abs(v) < ys * 1e-6 ? 0 : v, ys >= 1 ? 0 : ys >= 0.1 ? 1 : 2), padL - 6, y);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'alphabetic';
    const xs = niceStep(t1 - t0, Math.max(2, Math.floor(pw / 70)));
    for (let t = Math.ceil(t0 / xs) * xs; t <= t1 + 1e-9; t += xs) {
      ctx.fillStyle = ink('--muted');
      ctx.fillText(`${fmt(t, xs >= 1 ? 0 : xs >= 0.1 ? 1 : 2)} s`, X(t), h - 4);
    }

    // series
    ctx.save();
    ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
    ctx.lineJoin = 'round';
    for (const s of this.series) {
      const col = this.columns[s.key];
      if (!col) continue;
      ctx.strokeStyle = ink(s.color);
      ctx.lineWidth = s.dashed ? 1.5 : 2;
      ctx.setLineDash(s.dashed ? [5, 4] : []);
      ctx.beginPath();
      let pen = false;
      for (let i = i0; i < col.length; i++) {
        const v = col[i];
        if (!Number.isFinite(v)) { pen = false; continue; }
        if (pen) ctx.lineTo(X(times[i]), Yp(v)); else { ctx.moveTo(X(times[i]), Yp(v)); pen = true; }
      }
      ctx.stroke();
    }
    ctx.restore();
    ctx.setLineDash([]);

    // hover: crosshair + values at the nearest sample
    if (this.hoverX !== null && this.hoverX >= padL && this.hoverX <= padL + pw) {
      const th = t0 + ((this.hoverX - padL) / pw) * (t1 - t0);
      let k = i0;
      while (k < times.length - 1 && times[k] < th) k++;
      const x = Math.round(X(times[k])) + 0.5;
      ctx.strokeStyle = ink('--muted');
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph); ctx.stroke();
      for (const s of this.series) {
        const v = this.columns[s.key]?.[k];
        if (!Number.isFinite(v)) continue;
        ctx.fillStyle = ink(s.color);
        ctx.beginPath(); ctx.arc(x, Yp(v), 3.5, 0, 2 * Math.PI); ctx.fill();
        ctx.strokeStyle = ink('--panel'); ctx.lineWidth = 1.5; ctx.stroke();
      }
      this.tooltip.replaceChildren(
        el('div', { class: 'tip-time' }, `t = ${fmt(times[k], 2)} s`),
        ...this.series.map((s) => el('div', { class: 'tip-row' },
          el('i', { class: `swatch${s.dashed ? ' dashed' : ''}`, style: { '--c': `var(${s.color})` } }),
          el('span', {}, s.label), el('b', {}, `${fmt(this.columns[s.key]?.[k] ?? NaN, 3)}${this.unit ? ` ${this.unit}` : ''}`))));
      this.tooltip.hidden = false;
      this.tooltip.style.left = `${Math.min(Math.max(x + 10, 0), w - 170)}px`;
      this.tooltip.style.top = `${padT}px`;
    }
  }
}
