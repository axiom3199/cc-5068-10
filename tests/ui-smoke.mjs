// Headless UI smoke test. Launches Chromium against the running server, drives the
// real page through its documented window.__doa surface, and asserts that the UI is
// fed by an actual solve (not a canned arrow):
//   1. page boots with no console/page errors and a real result appears
//   2. a 3D array reports a bearing close to the world truth (via array->world)
//   3. a horizontal planar array reports an explicit ambiguity with >=2 candidates
//   4. physically rewiring channels while sending the channelMap keeps the answer
//   5. removing the source yields an explicit "insufficient" rejection
//   6. clicking the probe button turns the camera and re-analyses
//   7. probe + cross-pose consistency prunes the mirror branch
//   8. the follow loop slews the camera toward the confirmed source
//   9. an injected channel swap is diagnosed, localized, and confirmed by re-run
//  10. an injected mic dropout is told apart from a swap; logs stay bounded
//
// Run with the server already up:  node tests/ui-smoke.mjs [url]

import { chromium } from 'playwright';

const URL = process.argv[2] ?? 'http://127.0.0.1:4173/';
let failures = 0;
const log = [];
function check(name, ok, detail = '') {
  log.push(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? `  (${detail})` : ''}`);
  if (!ok) failures++;
}

const browser = await chromium.launch({
  args: ['--no-sandbox', '--autoplay-policy=no-user-gesture-required'],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 1600 } });

const consoleErrors = [];
page.on('console', (m) => { if (m.type() === 'error') consoleErrors.push(m.text()); });
page.on('pageerror', (e) => consoleErrors.push(String(e)));

await page.goto(URL, { waitUntil: 'networkidle' });
await page.waitForFunction(() => window.__doa && window.__doaLastResult, null, { timeout: 20000 });

const read = () => page.evaluate(() => {
  const r = window.__doaLastResult;
  const cw = window.__doa.candidatesWorld();
  const rank1 = cw.find((c) => c.rank === 1) ?? null;
  return {
    status: r.result.status,
    reason: r.result.reason,
    action: r.decision.action,
    reasonKey: r.decision.reasonKey,
    candidates: r.result.candidates.map((c) => ({
      azDeg: c.azDeg, elDeg: c.elDeg, plausible: c.plausible, isMirror: c.isMirror,
    })),
    worldTruth: window.__doa.state.worldTruth,
    solverWorld: rank1 ? [Math.sin(rank1.azDeg * Math.PI / 180) * Math.cos(rank1.elDeg * Math.PI / 180), Math.sin(rank1.elDeg * Math.PI / 180), Math.cos(rank1.azDeg * Math.PI / 180) * Math.cos(rank1.elDeg * Math.PI / 180)] : null,
    worldBest: r.decision.bearing?.world ?? null,
    pose: { panDeg: window.__doa.state.camera.panDeg, tiltDeg: window.__doa.state.camera.tiltDeg },
    channelMap: r.analysis.metadata.channelMap,
    frameLength: r.analysis.metadata.frameLength,
    pairs: r.analysis.tdoa.pairs.length,
    domStatus: document.getElementById('statusPill').textContent,
    summaryText: document.getElementById('resultSummary').textContent,
    candRows: document.querySelectorAll('#candidateList .cand').length,
  };
});

// Re-observe once at the same pose so the tracker's confirmFrames gate is satisfied
// (first frame of a new scene is intentionally a hold).
const settle = () => page.evaluate(() => window.__doa.runAnalysis()).then(() => new Promise((r) => setTimeout(r, 60)));

const angular = (a, b) => {
  const r = Math.acos(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))) * 180 / Math.PI;
  return r;
};

// --- 1. boot -----------------------------------------------------------------
let s = await read();
check('page boots and produces a real solve', s.status && s.pairs > 0 && s.frameLength > 0,
  `status=${s.status} pairs=${s.pairs} frame=${s.frameLength}`);
check('no console/page errors on boot', consoleErrors.length === 0, consoleErrors.slice(0, 3).join(' | '));
check('result panel rendered candidate rows', s.candRows > 0, `rows=${s.candRows}`);

// --- 2. 3D array accuracy against world truth --------------------------------
await page.evaluate(() => window.__doa.setScene({
  layout: 'tetra', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 55, elDeg: 18, distanceM: null, snrDb: 26, present: true },
  window: { startSec: 0.1, lengthSec: 0.12 },
}));
await settle();
s = await read();
const truthDir = [Math.sin(55 * Math.PI / 180) * Math.cos(18 * Math.PI / 180), Math.sin(18 * Math.PI / 180), Math.cos(55 * Math.PI / 180) * Math.cos(18 * Math.PI / 180)];
const err3d = s.solverWorld ? angular(s.solverWorld, truthDir) : 999;
check('3D array resolves to a bearing near world truth', s.status === 'resolved' && err3d < 6,
  `status=${s.status} err=${err3d.toFixed(2)}° action=${s.action}`);

// --- 3. planar array ambiguity is surfaced, not forced -----------------------
await page.evaluate(() => window.__doa.setScene({
  layout: 'square', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 60, elDeg: 35, distanceM: null, snrDb: 26, present: true },
}));
s = await read();
const planCands = s.candidates.filter((c) => c.plausible);
check('planar array keeps multiple candidates (no forced unique angle)',
  s.status === 'ambiguous' && planCands.length >= 2,
  `status=${s.status} cands=${planCands.length} reason=${(s.reason || '').slice(0, 48)}`);
check('planar ambiguity puts the camera on hold', s.action === 'hold', `action=${s.action} key=${s.reasonKey}`);

// --- 4. channel rewire is undone by the explicit channelMap ------------------
await page.evaluate(() => window.__doa.setScene({
  layout: 'tetra', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 70, elDeg: 12, distanceM: null, snrDb: 26, present: true },
  wiring: [2, 0, 3, 1],
}));
await settle();
s = await read();
const truthDir2 = [Math.sin(70 * Math.PI / 180) * Math.cos(12 * Math.PI / 180), Math.sin(12 * Math.PI / 180), Math.cos(70 * Math.PI / 180) * Math.cos(12 * Math.PI / 180)];
const errWired = s.solverWorld ? angular(s.solverWorld, truthDir2) : 999;
check('recorded channel order scrambled, map restores the correct bearing',
  s.channelMap.join(',') === '2,0,3,1' && errWired < 8,
  `map=[${s.channelMap}] err=${errWired.toFixed(2)}°`);

// --- 5. missing evidence is an explicit rejection ----------------------------
await page.evaluate(() => window.__doa.setScene({
  layout: 'tetra', source: { present: false }, wiring: null,
}));
s = await read();
check('silent input is judged insufficient and held',
  s.status === 'insufficient' && s.action === 'hold',
  `status=${s.status} action=${s.action}`);
check('insufficient result shows rejection text in the DOM',
  /证据不足/.test(s.summaryText), s.summaryText.slice(0, 40));

// --- 6. probe turns the camera ------------------------------------------------
await page.evaluate(() => window.__doa.setScene({
  layout: 'square', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 60, elDeg: 35, distanceM: null, snrDb: 26, present: true },
}));
const poseBefore = (await read()).pose;
await page.click('#probeBtn');
await page.waitForFunction(() => {
  const p = window.__doa.state.camera;
  return Math.abs(p.tiltDeg) > 8 || Math.abs(((p.panDeg + 180) % 360) - 180) > 8;
}, null, { timeout: 8000 });
await page.waitForTimeout(1200);
const poseAfter = (await read()).pose;
const moved = Math.abs(poseAfter.tiltDeg - poseBefore.tiltDeg) > 8
  || Math.abs(((poseAfter.panDeg - poseBefore.panDeg + 540) % 360) - 180) > 8;
check('probe button reorients the camera to separate the mirror', moved,
  `pan ${poseBefore.panDeg.toFixed(1)}→${poseAfter.panDeg.toFixed(1)}, tilt ${poseBefore.tiltDeg.toFixed(1)}→${poseAfter.tiltDeg.toFixed(1)}`);

// --- 7. probe + cross-pose consistency excludes the mirror branch ------------
await page.evaluate(() => window.__doa.setScene({
  layout: 'square', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 60, elDeg: 35, distanceM: null, snrDb: 28, present: true },
}));
await page.evaluate(() => window.__doa.runAnalysis());
const preP = await page.evaluate(() => {
  const t = window.__doa.state.tracker;
  return {
    status: window.__doaLastResult.result.status,
    action: window.__doaLastResult.decision.action,
    branches: t ? t.tracks.map((x) => +x.elDeg.toFixed(1)).sort((a, b) => a - b) : [],
  };
});
check('before probe the planar mirror is held (2 branches, no slew)',
  preP.status === 'ambiguous' && preP.action === 'hold' && preP.branches.length >= 2,
  `status=${preP.status} action=${preP.action} elBranches=[${preP.branches}]`);

await page.click('#probeBtn');
await page.waitForFunction(() => Math.abs(window.__doa.state.camera.tiltDeg) > 12
  || Math.abs(((window.__doa.state.camera.panDeg + 180) % 360) - 180) > 12, null, { timeout: 8000 });
await page.waitForTimeout(1500);
const postP = await page.evaluate(() => {
  const t = window.__doa.state.tracker;
  return {
    action: window.__doaLastResult.decision.action,
    reasonKey: window.__doaLastResult.decision.reasonKey,
    consistent: t?.consistentWorld?.() ?? null,
    branches: t ? t.tracks.map((x) => +x.elDeg.toFixed(1)) : [],
  };
});
const lockedEl = postP.consistent ? Math.asin(Math.max(-1, Math.min(1, postP.consistent[1]))) * 180 / Math.PI : NaN;
check('after probe the true branch (el +35) is locked and the mirror pruned',
  postP.action === 'slew' && Math.abs(lockedEl - 35) < 4 && postP.branches.every((e) => e > 0),
  `action=${postP.action} key=${postP.reasonKey} lockedEl=${lockedEl.toFixed(1)}° tracks=[${postP.branches.join(',')}]`);

// --- 8. follow loop actually slews the camera --------------------------------
await page.evaluate(() => window.__doa.setScene({
  layout: 'tetra', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: -50, elDeg: 10, distanceM: null, snrDb: 28, present: true },
}));
await page.evaluate(() => window.__doa.setFollow(true));
await page.evaluate(() => window.__doa.setScene({
  layout: 'tetra', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: -50, elDeg: 10, distanceM: null, snrDb: 28, present: true },
}));
await page.evaluate(() => window.__doa.runAnalysis());
await page.waitForTimeout(400);
await page.evaluate(() => window.__doa.runAnalysis());
await page.waitForFunction(() => Math.abs((((window.__doa.state.camera.panDeg + 180) % 360) - 180) + 50) < 25, null, { timeout: 10000 }).catch(() => {});
const followPose = (await read()).pose;
const near = Math.abs((((followPose.panDeg + 180) % 360) - 180) + 50) < 25;
check('follow loop slews the camera toward the confirmed source', near,
  `pan=${followPose.panDeg.toFixed(1)}° (target ≈ -50°)`);

// --- 9. injected channel swap is diagnosed, localized, and re-run-verified ----
await page.evaluate(() => window.__doa.setScene({
  layout: 'tetra', camera: { panDeg: 0, tiltDeg: 0 },
  source: { azDeg: 70, elDeg: 15, distanceM: null, snrDb: 28, present: true },
  wiring: null,
}));
await settle(); // establish the trusted reference on clean frames
const swapDiag = await page.evaluate(() => {
  window.__doa.injectFault({ type: 'swap', mics: [1, 2] });
  const events = window.__doa.jumpEvents();
  const ev = events.find((e) => e.kind === 'anomaly');
  const verify = ev ? window.__doa.verifyJump(ev.id) : null;
  const alert = document.getElementById('jumpAlert').textContent;
  const detailShown = (() => {
    if (!ev) return false;
    window.__doa.selectJumpEvent(ev.id);
    return document.getElementById('jumpEventDetail').textContent.length > 20;
  })();
  return ev ? {
    cause: ev.cause, stage: ev.stage, mics: ev.implicatedMics,
    conclusion: verify?.conclusion ?? null,
    steps: verify?.steps?.map((x) => x.step) ?? [],
    alert, detailShown,
  } : { cause: null, alert };
});
check('injected channel swap raises a channel-swap anomaly localized to M1/M2 (tdoa stage)',
  swapDiag.cause === 'channel-swap' && swapDiag.stage === 'tdoa'
  && swapDiag.mics?.join(',') === '1,2',
  `cause=${swapDiag.cause} stage=${swapDiag.stage} mics=[${swapDiag.mics ?? ''}]`);
check('swap diagnosis is confirmed by re-running the recorded evidence',
  swapDiag.conclusion === 'confirmed',
  `conclusion=${swapDiag.conclusion} steps=[${(swapDiag.steps ?? []).join('>')}]`);
check('the jump panel surfaces the anomaly and its detail in the DOM',
  /异常/.test(swapDiag.alert) && swapDiag.detailShown,
  `alert="${(swapDiag.alert ?? '').slice(0, 40)}" detail=${swapDiag.detailShown}`);

// --- 10. injected mic dropout is told apart from a swap -----------------------
const dropDiag = await page.evaluate(() => {
  window.__doa.injectFault({ type: 'drop', mic: 2 });
  const ev = window.__doa.jumpEvents().find((e) => e.kind === 'anomaly' && e.cause !== 'channel-swap');
  const stats = window.__doa.jumpStats();
  return ev ? {
    cause: ev.cause, stage: ev.stage, mics: ev.implicatedMics,
    events: stats.events, maxEvents: stats.maxEvents, segmentsHeld: stats.segmentsHeld,
  } : { cause: null };
});
check('injected mic dropout is diagnosed as mic-dropout on M2, not a swap',
  dropDiag.cause === 'mic-dropout' && dropDiag.mics?.join(',') === '2',
  `cause=${dropDiag.cause} mics=[${dropDiag.mics ?? ''}]`);
check('diagnostic log stays within its budget',
  dropDiag.events <= dropDiag.maxEvents && dropDiag.segmentsHeld <= 4,
  `events=${dropDiag.events}/${dropDiag.maxEvents} segments=${dropDiag.segmentsHeld}`);

await browser.close();

console.log(log.join('\n'));
console.log(`\n${failures === 0 ? 'UI SMOKE PASS' : 'UI SMOKE FAIL'} — ${log.length} checks, ${failures} failed`);
process.exit(failures === 0 ? 0 : 1);
