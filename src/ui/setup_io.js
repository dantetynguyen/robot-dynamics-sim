// setup_io.js — save and load the whole setup (parameters + edited equations)
// as a JSON file or through the clipboard.

import { Simulation } from '../sim/simulation.js';
import { button, el } from './dom.js';

export function setupControls(app) {
  const status = el('p', { class: 'status-line' });
  const say = (text, cls = '') => { status.className = `status-line ${cls}`; status.textContent = text; };
  const json = () => JSON.stringify(app.sim.toJSON(), null, 1);
  const load = (text) => {
    let data;
    try { data = JSON.parse(text); } catch (e) { say('That is not valid JSON.', 'bad'); return; }
    const problems = app.loadSetup(data);
    say(problems.length ? `Not loaded: ${problems.join('; ')}.` : 'Setup loaded.', problems.length ? 'bad' : 'ok');
  };
  const paste = el('textarea', { rows: 3, placeholder: 'Paste a saved setup here, then press Load pasted setup', style: { width: '100%' } });
  const file = el('input', { type: 'file', accept: '.json,application/json' });
  file.addEventListener('change', async () => { if (file.files[0]) load(await file.files[0].text()); file.value = ''; });
  return el('div', {},
    el('p', { class: 'note' }, 'A setup holds every parameter and every edited equation. It is also kept in this browser between visits.'),
    el('div', { class: 'row' },
      button('Download setup file', () => {
        try {
          const a = el('a', { href: URL.createObjectURL(new Blob([json()], { type: 'application/json' })), download: 'robot-dynamics-setup.json' });
          document.body.append(a); a.click(); a.remove();
          say('Download started. If nothing happened, use Copy setup instead.');
        } catch (e) { say('Downloads are blocked here. Use Copy setup instead.', 'warn'); }
      }, 'small'),
      button('Copy setup', async () => {
        try { await navigator.clipboard.writeText(json()); say('Setup copied to the clipboard.', 'ok'); }
        catch (e) { paste.value = json(); paste.select(); say('Clipboard is blocked here. The setup is in the box below; copy it from there.', 'warn'); }
      }, 'small')),
    el('div', { class: 'row' }, el('span', { class: 'field-label' }, 'Load a file'), file),
    paste,
    el('div', { class: 'row' },
      button('Load pasted setup', () => load(paste.value), 'small'),
      button('Restore all defaults', () => { load(JSON.stringify(new Simulation().toJSON())); say('Defaults restored.', 'ok'); }, 'small')),
    status);
}
