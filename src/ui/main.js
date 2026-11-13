// main.js — entry point for the browser.
import { App } from './app.js';

const root = document.getElementById('app');
try {
  window.app = new App(root); // exposed for debugging from the console
} catch (e) {
  root.innerHTML = '';
  const p = document.createElement('p');
  p.className = 'boot';
  p.textContent = `The simulator could not start: ${e.message}`;
  root.append(p);
  throw e;
}
