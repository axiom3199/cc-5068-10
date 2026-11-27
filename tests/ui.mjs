// Runs the headless UI smoke test against an ephemeral server it starts itself, so
// the check is self-contained (no manual `npm start` needed).
//
// Browser availability: Chromium is launched through Playwright. On a machine that
// is missing the usual Chromium shared libraries, point LD_LIBRARY_PATH at a local
// sysroot (see README "Browser-based UI check").

import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { setTimeout as delay } from 'node:timers/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const port = 4319 + Math.floor(Math.random() * 200);

const server = spawn(process.execPath, [resolve(root, 'server.mjs'), '--port', String(port)], {
  cwd: root,
  stdio: ['ignore', 'pipe', 'inherit'],
});
await once(server.stdout, 'data');

const url = `http://127.0.0.1:${port}/`;
const test = spawn(process.execPath, [resolve(root, 'tests/ui-smoke.mjs'), url], {
  cwd: root,
  stdio: 'inherit',
});
const [code] = await once(test, 'exit');
server.kill();
await delay(50);
if (code !== 0) {
  console.error(`\nUI smoke test failed (exit ${code}).`);
  process.exit(code ?? 1);
}
