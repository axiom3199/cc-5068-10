import { chromium } from 'playwright';

const URL = process.argv[2] ?? 'http://127.0.0.1:4173/';
const browser = await chromium.launch({ args: ['--no-sandbox'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 1000 } });
page.on('pageerror', (e) => console.error('PAGEERROR', String(e)));
await page.goto(URL, { waitUntil: 'networkidle' });
await page.waitForFunction(() => window.__doa && window.__doaLastResult, null, { timeout: 20000 });

const snap = () => page.evaluate(() => {
  const r = window.__doaLastResult;
  return {
    status: r.result.status,
    action: r.decision.action,
    reasonKey: r.decision.reasonKey,
    tilt: window.__doa.state.camera.tiltDeg,
    pan: window.__doa.state.camera.panDeg,
    consistent: window.__doa.state.tracker?.consistentWorld?.() ?? null,
    tracks: window.__doa.state.tracker?.tracks?.map((t) => ({ az: t.azDeg, el: t.elDeg, w: +t.weight.toFixed(2), hits: t.hits })) ?? [],
  };
});

await page.evaluate(() => window.__doa.setScene({
  layout: 'square', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 60, elDeg: 35, distanceM: null, snrDb: 28, present: true },
}));
await page.evaluate(() => window.__doa.runAnalysis());
console.log('before probe:', JSON.stringify(await snap()));

await page.click('#probeBtn');
await page.waitForFunction(() => Math.abs(window.__doa.state.camera.tiltDeg) > 12
  || Math.abs(((window.__doa.state.camera.panDeg + 180) % 360) - 180) > 12, null, { timeout: 8000 });
await page.waitForTimeout(1500);
console.log('after probe:', JSON.stringify(await snap()));
await page.evaluate(() => window.__doa.runAnalysis());
await page.waitForTimeout(200);
console.log('after 2nd analyse:', JSON.stringify(await snap()));

await browser.close();
