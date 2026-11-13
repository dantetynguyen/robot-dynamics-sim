// dom.js — tiny helpers for building the interface without a framework.

// el('div', { class: 'row', onclick: fn }, child, 'text', …)
export function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs ?? {})) {
    if (v === null || v === undefined || v === false) continue;
    if (k.startsWith('on') && typeof v === 'function') node.addEventListener(k.slice(2), v);
    else if (k === 'class') node.className = v;
    else if (k === 'html') node.innerHTML = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(node.style, v);
    else node.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children.flat(Infinity)) {
    if (c === null || c === undefined || c === false) continue;
    node.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return node;
}

export function clear(node) {
  while (node.firstChild) node.removeChild(node.firstChild);
  return node;
}

// A labelled control on one line.
export function field(label, control, hint) {
  return el('label', { class: 'field' }, el('span', { class: 'field-label' }, label), control, hint ? el('span', { class: 'field-hint' }, hint) : null);
}

// Number box. onChange is called only with finite numbers inside [min, max];
// anything else marks the box invalid and leaves the simulation untouched.
export function numberInput({ value, min = -Infinity, max = Infinity, step = 'any', onChange, digits = 4, title }) {
  const input = el('input', { type: 'number', class: 'num', step, title, value: trim(value, digits) });
  input.addEventListener('input', () => {
    const v = input.value.trim() === '' ? NaN : Number(input.value);
    const ok = Number.isFinite(v) && v >= min && v <= max;
    input.classList.toggle('invalid', !ok);
    input.title = ok ? (title ?? '') : `Enter a number${Number.isFinite(min) ? ` from ${min}` : ''}${Number.isFinite(max) ? ` to ${max}` : ''}`;
    if (ok) onChange(v);
  });
  return input;
}

// A row of number boxes editing an array in place.
export function vectorInput({ values, onChange, min, max, digits = 4, labels }) {
  const current = values.slice();
  return el('span', { class: 'vec' }, current.map((v, i) => numberInput({
    value: v, min, max, digits, title: labels ? labels[i] : undefined,
    onChange: (x) => { current[i] = x; onChange(current.slice(), i); },
  })));
}

export function slider({ min, max, step, value, onInput, format = (v) => v }) {
  const out = el('output', { class: 'slider-value' }, format(value));
  const input = el('input', { type: 'range', min, max, step, value });
  input.addEventListener('input', () => { const v = Number(input.value); out.textContent = format(v); onInput(v); });
  const wrap = el('span', { class: 'slider' }, input, out);
  wrap.set = (v) => { input.value = v; out.textContent = format(v); };
  return wrap;
}

export function select(options, value, onChange) {
  const s = el('select', {}, options.map(([v, label]) => el('option', { value: v, selected: String(v) === String(value) }, label)));
  s.addEventListener('change', () => onChange(s.value));
  return s;
}

export function checkbox(label, checked, onChange, hint) {
  const input = el('input', { type: 'checkbox', checked });
  input.addEventListener('change', () => onChange(input.checked));
  return el('label', { class: 'check', title: hint }, input, el('span', {}, label));
}

export function button(label, onClick, cls = '') {
  return el('button', { type: 'button', class: `btn ${cls}`, onclick: onClick }, label);
}

// Collapsible group of controls.
export function section(title, open, ...children) {
  return el('details', { class: 'section', open }, el('summary', {}, title), el('div', { class: 'section-body' }, children));
}

function trim(v, digits) {
  if (!Number.isFinite(v)) return '';
  return String(Number(v.toPrecision(Math.max(digits, 1))));
}
