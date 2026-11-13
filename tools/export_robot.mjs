// export_robot.mjs — write the default robot, with shapes resolved to inertia
// matrices, to reference/robot_default.json for the independent Python
// reference. Run via `npm run reference`.
import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { PRESETS, DEFAULT_PRESET } from '../src/robot/presets.js';
import { referenceSpec } from '../src/robot/model.js';

const out = fileURLToPath(new URL('../reference/robot_default.json', import.meta.url));
writeFileSync(out, JSON.stringify(referenceSpec(PRESETS[DEFAULT_PRESET]()), null, 1));
console.log('wrote', out);
