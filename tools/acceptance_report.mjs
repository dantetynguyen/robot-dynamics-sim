// acceptance_report.mjs — print every number recorded by the last test run.
// Usage: npm run acceptance      (runs the tests first)
import { readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const dir = fileURLToPath(new URL('../tests/results/', import.meta.url));
let lines = ['| Check | Measured | Requirement | Result |', '|---|---|---|---|'];
let fails = 0;
for (const file of readdirSync(dir).filter((f) => f.endsWith('.json')).sort()) {
  lines.push(`| **${file.replace('.json', '')}** | | | |`);
  for (const r of JSON.parse(readFileSync(dir + file, 'utf8'))) {
    if (!r.pass) fails++;
    const v = Math.abs(r.value) >= 1e-3 && Math.abs(r.value) < 1e4 ? r.value.toPrecision(4) : r.value.toExponential(2);
    lines.push(`| ${r.name}${r.note ? ` <br><sub>${r.note}</sub>` : ''} | ${v} ${r.unit} | ${r.dir} ${r.limit} | ${r.pass ? 'pass' : '**FAIL**'} |`);
  }
}
const text = lines.join('\n') + '\n';
writeFileSync(fileURLToPath(new URL('../tests/results/ACCEPTANCE.md', import.meta.url)), text);
console.log(text);
console.log(fails ? `${fails} check(s) FAILED` : 'all recorded checks passed');
