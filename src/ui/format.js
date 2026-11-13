// format.js — numbers, vectors and matrices as text / HTML / LaTeX.

export function fmt(x, digits = 3) {
  if (!Number.isFinite(x)) return x > 0 ? '∞' : x < 0 ? '−∞' : '—';
  const a = Math.abs(x);
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) return x.toExponential(2).replace('-', '−');
  const s = x.toFixed(digits);
  return (Object.is(Number(s), -0) || Number(s) === 0 ? s.replace('-', '') : s).replace('-', '−');
}

export const fmtVec = (v, digits = 3) => `[${v.map((x) => fmt(x, digits)).join(', ')}]`;

// A matrix (or vector, shown as a column) as an HTML table with aligned figures.
export function matrixTable(M, digits = 3) {
  const rows = Array.isArray(M[0]) ? M : M.map((x) => [x]);
  const table = document.createElement('table');
  table.className = 'matrix';
  for (const row of rows) {
    const tr = table.insertRow();
    for (const v of row) tr.insertCell().textContent = fmt(v, digits);
  }
  return table;
}

// LaTeX bmatrix for KaTeX.
export function texMatrix(M, digits = 3) {
  const rows = Array.isArray(M[0]) ? M : M.map((x) => [x]);
  const cell = (x) => {
    const s = fmt(x, digits).replace('−', '-');
    return s.includes('e') ? s.replace(/e([+-]?)(\d+)/, (_, sg, d) => `\\!\\times\\!10^{${sg === '-' ? '-' : ''}${Number(d)}}`) : s;
  };
  return `\\begin{bmatrix}${rows.map((r) => r.map(cell).join('&')).join('\\\\')}\\end{bmatrix}`;
}

// Render LaTeX into an element with the vendored KaTeX (loaded as a global).
// Falls back to the raw source if KaTeX is unavailable.
export function tex(source, display = false) {
  const span = document.createElement(display ? 'div' : 'span');
  span.className = display ? 'tex tex-display' : 'tex';
  try {
    window.katex.render(source, span, { displayMode: display, throwOnError: false });
  } catch (e) {
    span.textContent = source;
  }
  return span;
}

export const DEG = Math.PI / 180;
