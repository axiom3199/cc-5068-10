import { chromium } from 'playwright';

const URL = process.argv[2] ?? 'http://127.0.0.1:4173/';
const browser = await chromium.launch({ args: ['--no-sandbox'] });
const page = await browser.newPage({ viewport: { width: 1500, height: 1700 }, deviceScaleFactor: 1 });
page.on('pageerror', (e) => console.error('PAGEERROR', String(e)));
await page.goto(URL, { waitUntil: 'networkidle' });
await page.waitForFunction(() => window.__doa && window.__doaLastResult, null, { timeout: 20000 });

// 3D resolved scene
await page.evaluate(() => window.__doa.setScene({
  layout: 'tetra', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 55, elDeg: 18, distanceM: null, snrDb: 26, present: true },
  window: { startSec: 0.1, lengthSec: 0.12 },
}));
await page.evaluate(() => window.__doa.runAnalysis());
await page.waitForTimeout(300);
await page.screenshot({ path: '/tmp/shot-resolved.png', fullPage: true });

// planar ambiguity, after a probe tilt
await page.evaluate(() => window.__doa.setScene({
  layout: 'square', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 60, elDeg: 35, distanceM: null, snrDb: 26, present: true },
}));
await page.evaluate(() => window.__doa.runAnalysis());
await page.evaluate(() => window.__doa.setScene({ camera: { panDeg: 0, tiltDeg: 16 } }));
await page.evaluate(() => window.__doa.runAnalysis());
await page.evaluate(() => window.__doa.runAnalysis());
await page.waitForTimeout(300);
await page.screenshot({ path: '/tmp/shot-planar.png', fullPage: true });

await browser.close();
console.log('shots written');
