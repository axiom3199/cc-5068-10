// Zero-dependency static server for the direction-finding UI.
// Usage: node server.mjs [--port 4173] [--host 127.0.0.1]
// Serves the repository root so ES modules in src/ load with correct MIME types.

import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { extname, join, normalize, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(fileURLToPath(new URL('.', import.meta.url)));

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.wav': 'audio/wav',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.ico': 'image/x-icon',
  '.map': 'application/json',
};

function parseArgs(argv) {
  const out = { port: 4173, host: '127.0.0.1' };
  for (let i = 2; i < argv.length; i++) {
    if (argv[i] === '--port' && argv[i + 1]) out.port = Number(argv[++i]);
    else if (argv[i] === '--host' && argv[i + 1]) out.host = argv[++i];
  }
  return out;
}

const { port, host } = parseArgs(process.argv);

const server = createServer(async (req, res) => {
  try {
    const url = new URL(req.url, `http://${req.headers.host}`);
    let pathname = decodeURIComponent(url.pathname);
    if (pathname === '/' || pathname === '') pathname = '/index.html';
    const safe = normalize(pathname).replace(/^(\.\.[/\\])+/, '');
    const filePath = join(root, safe);
    if (!filePath.startsWith(root)) {
      res.writeHead(403).end('Forbidden');
      return;
    }
    const st = await stat(filePath).catch(() => null);
    if (!st || !st.isFile()) {
      res.writeHead(404, { 'content-type': 'text/plain; charset=utf-8' }).end('Not found');
      return;
    }
    const body = await readFile(filePath);
    res.writeHead(200, {
      'content-type': MIME[extname(filePath).toLowerCase()] ?? 'application/octet-stream',
      'cache-control': 'no-cache',
    });
    res.end(body);
  } catch (e) {
    res.writeHead(500, { 'content-type': 'text/plain; charset=utf-8' }).end(`Server error: ${e.message}`);
  }
});

server.listen(port, host, () => {
  console.log(`Acoustic direction lab running at http://${host}:${port}/`);
  console.log('Open that URL in a browser. Ctrl+C to stop.');
});
