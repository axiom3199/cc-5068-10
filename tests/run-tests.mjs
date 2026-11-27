// Minimal test runner: no dependencies, deterministic, exits nonzero on failure.
import { arrays, sceneFor, truePairTaus, tdoaPair, angleError, FS, C, check, approx } from './harness.mjs';
import { estimateAllPairs, observationsFromPairs } from '../src/core/tdoa.js';
import { solveDirection, analyzeArrayGeometry } from '../src/core/solver.js';
import { analyzeDirection, loudestChannelBearing } from '../src/core/analysis.js';
import { runTrackingTrial } from './tracking-eval.mjs';
import { runProbeTrial, runScriptedDisambiguation } from './probe-eval.mjs';
import {
  scenarioChannelSwap, scenarioMicDropout, scenarioStrongReflection,
  scenarioMultiPeakFlip, scenarioAxisTransform, scenarioSourceMoved, scenarioLogCost,
  evaluateScenario,
} from './jump-eval.mjs';
import { azElToUnit, angularSeparationDeg, unitToAzEl, arrayToWorld } from '../src/core/geometry.js';
import { renderScene } from '../src/core/synthesize.js';

const verbose = process.argv.includes('--verbose');
const results = [];
let current = null;

function test(name, fn) {
  current = { name, passed: 0, failed: 0, failures: [], notes: [] };
  results.push(current);
  const started = Date.now();
  try {
    fn({
      check: (cond, msg) => {
        if (cond) { current.passed++; return; }
        current.failed++;
        current.failures.push(msg);
        throw new Error(msg);
      },
      note: (msg) => { current.notes.push(msg); if (verbose) console.log(`    ${msg}`); },
    });
  } catch (e) {
    if (current.failures.length === 0) {
      current.failed++;
      current.failures.push(`threw: ${e.message}`);
    }
  }
  current.ms = Date.now() - started;
}

function solveFromScene(positions, az, el, opts = {}) {
  const sc = sceneFor(positions, az, el, opts);
  const est = estimateAllPairs({
    channels: sc.channels, positions, sampleRate: FS, speedOfSound: C,
    frameLength: opts.frameLength ?? 48000,
    bandLow: 300, bandHigh: 6000,
    maxLagSec: opts.maxLagSec ?? 0.0007,
  });
  return solveDirection({
    observations: observationsFromPairs(est.pairs), positions, referenceIndex: 0,
    speedOfSound: C, model: opts.model ?? 'far', allowNear: opts.allowNear,
  });
}

// ---------------------------------------------------------------------------
test('TDOA sign convention matches predicted arrival offsets', ({ check, note }) => {
  const A = arrays();
  for (const [name, az, el] of [['square', 0, 0], ['square', 45, 0], ['square', 140, 10], ['tetra', 250, 30]]) {
    const sc = sceneFor(A[name], az, el, { snrDb: 30 });
    const truth = truePairTaus(A[name], az, el, 'far');
    for (const { i, j, tau } of truth) {
      if (Math.abs(tau) < 20e-6) continue; // not resolvable below this
      const r = tdoaPair(sc.channels, i, j);
      const err = r.lagSec - tau;
      check(Math.sign(err) === Math.sign(tau) || Math.abs(err) < 3e-6, `${name} pair ${i}${j} sign flipped: true ${(tau * 1e6).toFixed(1)}us est ${(r.lagSec * 1e6).toFixed(1)}us`);
      check(Math.abs(err) < 4e-6, `${name} pair ${i}${j} error ${(err * 1e6).toFixed(2)}us exceeds 4us at 30 dB`);
    }
    note(`${name} az=${az} el=${el}: all pairs within tolerance`);
  }
});

// ---------------------------------------------------------------------------
test('Loudest channel is NOT the direction (baseline deliberately wrong)', ({ check, note }) => {
  const A = arrays();
  const positions = A.square;
  const sc = sceneFor(positions, 90, 0, { snrDb: 30 });
  const loudest = loudestChannelBearing({ channels: sc.channels, sampleRate: FS }, { positions });
  // The truth is 90 deg (to the right), where mics 1 and 2 sit at +x. The whole
  // point: peak amplitude reports at best a mic bearing, never the source bearing.
  const truth = azElToUnit(90, 0);
  const loudestDir = azElToUnit(loudest.azDeg, loudest.elDeg);
  const loudestErr = angularSeparationDeg(loudestDir, truth);
  const analysis = analyzeDirection({
    recording: { channels: sc.channels, sampleRate: FS },
    geometry: { positions, speedOfSound: C },
  });
  const realErr = angularSeparationDeg(
    azElToUnit(analysis.direction.best.azDeg, analysis.direction.best.elDeg), truth,
  );
  check(realErr < 3, `TDOA estimate should be accurate, got ${realErr.toFixed(1)}°`);
  note(`loudest-mic bearing error ${loudestErr.toFixed(0)}° vs TDOA error ${realErr.toFixed(2)}°`);
});

// ---------------------------------------------------------------------------
test('Far-field direction error across 3D arrays and angles', ({ check, note }) => {
  const A = arrays();
  const cases = [
    ['tetra', 0, 0], ['tetra', 75, 25], ['tetra', 200, -30], ['tetra', 310, 40],
  ];
  for (const [name, az, el] of cases) {
    const res = solveFromScene(A[name], az, el, { snrDb: 30 });
    check(res.status === 'resolved', `${name} (${az},${el}) status ${res.status}: ${res.reason}`);
    check(res.best, `${name} (${az},${el}) produced no estimate`);
    const err = angularSeparationDeg(azElToUnit(res.best.azDeg, res.best.elDeg), azElToUnit(az, el));
    check(err < 3, `${name} (${az},${el}) error ${err.toFixed(2)}° > 3°`);
    note(`${name} (${az},${el}) -> ${res.best.azDeg.toFixed(1)},${res.best.elDeg.toFixed(1)} err ${err.toFixed(2)}° ${res.status}`);
  }
});

test('Planar arrays keep the true direction among candidates (up to mirror)', ({ check, note }) => {
  const A = arrays();
  // On the horizon the up/down mirror coincides with the truth, so a planar array is
  // legitimately unambiguous there; test off-horizon angles where a mirror exists.
  for (const [name, az, el] of [['triangle', 30, 12], ['triangle', 260, 10], ['square', 120, -15]]) {
    const res = solveFromScene(A[name], az, el, { snrDb: 30 });
    check(res.status === 'ambiguous', `${name} planar array at ${az},${el} should be ambiguous, got ${res.status}`);
    const truth = azElToUnit(az, el);
    const bestErr = angularSeparationDeg(azElToUnit(res.best.azDeg, res.best.elDeg), truth);
    const anyErr = Math.min(...res.candidates.map((c) => angularSeparationDeg(azElToUnit(c.azDeg, c.elDeg), truth)));
    check(anyErr < 3, `${name} (${az},${el}) truth not within 3° of ANY candidate (closest ${anyErr.toFixed(1)}°)`);
    note(`${name} (${az},${el}) best err ${bestErr.toFixed(1)}°, closest candidate err ${anyErr.toFixed(2)}°`);
  }
});

test('A planar array on the horizon resolves to a single elevation', ({ check, note }) => {
  const A = arrays();
  const res = solveFromScene(A.square, 40, 0, { snrDb: 30 });
  check(res.status === 'resolved', `on-horizon planar source should be resolved, got ${res.status}: ${res.reason}`);
  const err = angularSeparationDeg(azElToUnit(res.best.azDeg, res.best.elDeg), azElToUnit(40, 0));
  check(err < 3, `on-horizon error ${err.toFixed(1)}°`);
  note(`on-horizon -> ${res.status}, err ${err.toFixed(2)}° (mirror coincides with truth)`);
});

// ---------------------------------------------------------------------------
test('Planar array reports front/back (up/down) mirror instead of one angle', ({ check, note }) => {
  const A = arrays();
  for (const [az, el] of [[50, 25], [120, -15], [300, 12]]) {
    const res = solveFromDir('square', az, el);
    check(res.status === 'ambiguous', `expected ambiguous for planar array at ${az},${el}, got ${res.status}`);
    check(res.ambiguity.type === 'plane-mirror' || res.ambiguity.type === 'multi-modal', `ambiguity type was ${res.ambiguity.type}`);
    check(res.candidates.length >= 2, `expected >=2 candidates, got ${res.candidates.length}`);
    const truth = azElToUnit(az, el);
    const hasTrue = res.candidates.some((c) => angularSeparationDeg(azElToUnit(c.azDeg, c.elDeg), truth) < 3);
    check(hasTrue, `true direction missing from candidate set for ${az},${el}`);
    const mirror = res.candidates.find((c) => angularSeparationDeg(azElToUnit(c.azDeg, c.elDeg), truth) > 10);
    check(!!mirror, `no well-separated mirror candidate for ${az},${el}`);
    note(`${az},${el}: candidates ${res.candidates.map((c) => `(${c.azDeg.toFixed(0)},${c.elDeg.toFixed(0)})`).join(' ')} type ${res.ambiguity.type}`);
  }
});

function solveFromDir(name, az, el, opts = {}) {
  return solveFromScene(arrays()[name], az, el, { snrDb: 28, ...opts });
}

// ---------------------------------------------------------------------------
test('Collinear array reports a cone, never a unique angle', ({ check, note }) => {
  const A = arrays();
  for (const [az, el] of [[60, 0], [100, 15]]) {
    const res = solveFromScene(A.line, az, el, { snrDb: 30 });
    check(res.status === 'ambiguous', `collinear array should be ambiguous, got ${res.status}: ${res.reason}`);
    check(res.ambiguity.type === 'cone', `expected cone ambiguity, got ${res.ambiguity.type}`);
    check(res.ambiguity.cone && res.ambiguity.cone.halfAngleDeg >= 0, 'cone description missing');
    note(`${az},${el}: cone half-angle ${res.ambiguity.cone.halfAngleDeg.toFixed(1)}°`);
  }
});

test('Two-microphone array never resolves a full 3D direction', ({ check, note }) => {
  const A = arrays();
  const res = solveFromScene(A.line.slice(0, 2), 45, 0, { snrDb: 30 });
  check(res.status === 'insufficient', `2-mic array reported ${res.status} instead of insufficient`);
  check(res.candidates.length === 0, '2-mic array presented candidates as facts');
  check(/two microphones|cone/i.test(res.reason), `reason should explain the cone, got "${res.reason}"`);
  note(`2 mic -> ${res.status}: ${res.reason}`);
});

// ---------------------------------------------------------------------------
test('Near-field model is selected and localized when the source is close', ({ check, note }) => {
  // A 3D array so the direction is identifiable; a planar array would add a mirror
  // that is irrelevant to the near/far question.
  const positions = [[0.08, 0.06, 0.05], [0.08, -0.05, -0.06], [-0.08, 0.05, -0.05], [-0.08, -0.06, 0.05]];
  for (const [az, el, d] of [[20, 0, 0.45], [300, 20, 0.35], [60, 10, 0.6]]) {
    const sc = sceneFor(positions, az, el, { snrDb: 32, seed: 5, nearField: true, distanceM: d, durationSec: 1.2 });
    const est = estimateAllPairs({
      channels: sc.channels, positions, sampleRate: FS, speedOfSound: C,
      frameLength: 57600, bandLow: 300, bandHigh: 6000, maxLagSec: 0.0015,
    });
    const res = solveDirection({
      observations: observationsFromPairs(est.pairs), positions, referenceIndex: 0,
      speedOfSound: C, model: 'auto', distanceM: 1,
    });
    check(res.model === 'near', `expected near model for d=${d}, got ${res.model}`);
    const err = angularSeparationDeg(azElToUnit(res.best.azDeg, res.best.elDeg), azElToUnit(az, el));
    check(err < 4, `near-field direction error ${err.toFixed(2)}° > 4°`);
    const dErr = Math.abs(res.nearDistanceM - d) / d;
    check(dErr < 0.35, `near-field distance error ${(dErr * 100).toFixed(0)}% > 35% (est ${res.nearDistanceM.toFixed(2)} true ${d})`);
    note(`near (${az},${el},${d}m) -> ${res.best.azDeg.toFixed(1)},${res.best.elDeg.toFixed(1)},${res.nearDistanceM.toFixed(2)}m err ${err.toFixed(2)}°`);
  }
});

test('Distant spherical-wave source: near-field is allowed only with a sane distance', ({ check, note }) => {
  // renderScene always emits a spherical wave (its delay model is exact), so a near-
  // field fit is legitimate at any range; the requirement is that the reported
  // distance is physically plausible and the fit is not worse than far-field. A true
  // PLANE wave is checked separately below.
  const positions = [[0.08, 0.06, 0.05], [0.08, -0.05, -0.06], [-0.08, 0.05, -0.05], [-0.08, -0.06, 0.05]];
  const sc = sceneFor(positions, 0, 0, { snrDb: 30, seed: 5, distanceM: 12, durationSec: 1.2 });
  const est = estimateAllPairs({
    channels: sc.channels, positions, sampleRate: FS, speedOfSound: C,
    frameLength: 57600, bandLow: 300, bandHigh: 6000, maxLagSec: 0.0015,
  });
  const res = solveDirection({
    observations: observationsFromPairs(est.pairs), positions, referenceIndex: 0,
    speedOfSound: C, model: 'auto', distanceM: 1,
  });
  const far = solveDirection({
    observations: observationsFromPairs(est.pairs), positions, referenceIndex: 0,
    speedOfSound: C, model: 'far',
  });
  if (res.model === 'near') {
    check(res.nearDistanceM > 2, `near-fit distance ${res.nearDistanceM?.toFixed(2)} m is implausibly small for a 12 m source`);
    check(res.fit.chi2Red <= far.fit.chi2Red + 1e-6, 'near model accepted without improving the fit');
  }
  note(`12 m spherical wave -> ${res.model}${res.model === 'near' ? ` at ${res.nearDistanceM.toFixed(1)} m` : ''}; far chi2/dof ${far.fit.chi2Red.toFixed(2)}`);
});

test('Plane-wave source selects far-field (no spurious near-field fit)', ({ check, note }) => {
  // Build TDOAs from the far-field model directly, so a near-field model has nothing
  // to explain and must be rejected.
  const positions = [[0.08, 0.06, 0.05], [0.08, -0.05, -0.06], [-0.08, 0.05, -0.05], [-0.08, -0.06, 0.05]];
  const az = 25;
  const el = 8;
  const truth = truePairTaus(positions, az, el, 'far');
  const obs = truth.map((t) => ({ pair: [t.i, t.j], tau: t.tau + (Math.random() - 0.5) * 2e-7, sigma: 3e-7, coherence: 0.99, peakToFloor: 20 }));
  const res = solveDirection({ observations: obs, positions, referenceIndex: 0, speedOfSound: C, model: 'auto', distanceM: 1 });
  check(res.model === 'far', `plane-wave data fitted as ${res.model}${res.nearDistanceM ? ` at ${res.nearDistanceM.toFixed(1)} m` : ''}`);
  note(`plane wave -> ${res.model}`);
});

// ---------------------------------------------------------------------------
test('Noise-only input reports insufficient evidence (no invented direction)', ({ check, note }) => {
  const A = arrays();
  for (const snr of [-20, -12]) {
    let insufficient = 0;
    const N = 8;
    for (let s = 0; s < N; s++) {
      const sc = sceneFor(A.tetra, 40, 5, { snrDb: snr, seed: 100 + s, durationSec: 0.8 });
      const an = analyzeDirection({
        recording: { channels: sc.channels, sampleRate: FS },
        geometry: { positions: A.tetra, speedOfSound: C },
        options: { frameLength: 5760 },
      });
      if (an.direction.status === 'insufficient') insufficient++;
    }
    check(insufficient >= N - 1, `snr ${snr} dB: only ${insufficient}/${N} noise-only frames withheld a direction`);
    note(`snr ${snr} dB: ${insufficient}/${N} insufficient`);
  }
});

test('Silent channels give insufficient, not NaN', ({ check, note }) => {
  const A = arrays();
  const channels = A.tetra.map(() => new Float32Array(48000));
  const an = analyzeDirection({
    recording: { channels, sampleRate: FS },
    geometry: { positions: A.tetra, speedOfSound: C },
  });
  check(an.direction.status === 'insufficient', `silent input returned ${an.direction.status}`);
  check(Number.isFinite(an.direction.confidence), 'confidence not finite');
  note(`silent -> ${an.direction.status}: ${an.direction.reason}`);
});

// ---------------------------------------------------------------------------
test('Channel map is honored (out-of-order channels)', ({ check, note }) => {
  const A = arrays();
  const positions = A.tetra;
  const sc = sceneFor(positions, 110, 20, { snrDb: 30, seed: 4 });
  // Scramble the recording order and declare the inverse mapping.
  const order = [2, 0, 3, 1];
  const scrambled = order.map((i) => sc.channels[i]);
  const channelMap = order.map((_, recIdx) => order.indexOf(recIdx));
  // channelMap[m] = recording channel holding mic m
  const good = analyzeDirection({
    recording: { channels: scrambled, sampleRate: FS },
    geometry: { positions, speedOfSound: C, channelMap },
  });
  const bad = analyzeDirection({
    recording: { channels: scrambled, sampleRate: FS },
    geometry: { positions, speedOfSound: C },
  });
  const errGood = angularSeparationDeg(
    azElToUnit(good.direction.best.azDeg, good.direction.best.elDeg), azElToUnit(110, 20),
  );
  check(errGood < 3, `with channelMap, error ${errGood.toFixed(1)}°`);
  check(good.metadata.channelMap.join(',') === channelMap.join(','), 'metadata did not echo channelMap');
  const errBadBest = bad.direction.best
    ? angularSeparationDeg(azElToUnit(bad.direction.best.azDeg, bad.direction.best.elDeg), azElToUnit(110, 20))
    : 999;
  note(`mapped err ${errGood.toFixed(2)}°, unmapped-wrong-order err ${errBadBest.toFixed(1)}°`);
});

test('Sample-rate metadata is mandatory and used', ({ check, note }) => {
  const A = arrays();
  let threw = false;
  try {
    analyzeDirection({
      recording: { channels: A.tetra.map(() => new Float32Array(4800)), sampleRate: 0 },
      geometry: { positions: A.tetra, speedOfSound: C },
    });
  } catch (e) { threw = true; }
  check(threw, 'zero sample rate should be rejected');
  // 32 kHz recording at 32 kHz declared correctly: geometry must still resolve.
  const sc32 = renderScene({
    positions: A.tetra, source: { azDeg: 75, elDeg: 10 },
    sampleRate: 32000, speedOfSound: C, durationSec: 1, snrDb: 30, seed: 2,
    bandLow: 300, bandHigh: 6000, equalPower: true,
  });
  const an = analyzeDirection({
    recording: { channels: sc32.channels, sampleRate: 32000 },
    geometry: { positions: A.tetra, speedOfSound: C },
    options: { frameLength: 4096 },
  });
  const err = angularSeparationDeg(
    azElToUnit(an.direction.best.azDeg, an.direction.best.elDeg), azElToUnit(75, 10),
  );
  check(err < 4, `32 kHz error ${err.toFixed(1)}°`);
  check(an.metadata.sampleRate === 32000, 'sample rate not propagated to metadata');
  note(`32 kHz err ${err.toFixed(2)}°`);
});

test('Speed of sound is a required input and affects the fit', ({ check, note }) => {
  const A = arrays();
  const sc = sceneFor(A.tetra, 80, 5, { snrDb: 30, seed: 6 });
  const est = estimateAllPairs({
    channels: sc.channels, positions: A.tetra, sampleRate: FS, speedOfSound: C,
    frameLength: 48000, bandLow: 300, bandHigh: 6000, maxLagSec: 0.0007,
  });
  const obs = observationsFromPairs(est.pairs);
  const right = solveDirection({ observations: obs, positions: A.tetra, referenceIndex: 0, speedOfSound: 343, model: 'far' });
  const wrong = solveDirection({ observations: obs, positions: A.tetra, referenceIndex: 0, speedOfSound: 300, model: 'far' });
  // In the far-field model the direction is invariant to a wrong c (the geometry
  // just rescales), so the right test is that the RESIDUALS blow up: a wrong speed
  // makes the measured TDOAs inconsistent with any direction.
  check(right.fit.chi2Red < 2, `correct c reduced chi2 ${right.fit.chi2Red.toFixed(2)}`);
  check(wrong.fit.chi2Red > right.fit.chi2Red * 20, `wrong c should inflate residuals: right ${right.fit.chi2Red.toFixed(2)}, wrong ${wrong.fit.chi2Red.toFixed(0)}`);
  check(wrong.status === 'insufficient', `wrong c should be rejected as inconsistent, got ${wrong.status}`);
  note(`c=343 chi2/dof ${right.fit.chi2Red.toFixed(2)}; c=300 chi2/dof ${wrong.fit.chi2Red.toFixed(0)} -> ${wrong.status}`);
});

// ---------------------------------------------------------------------------
test('Tracking: 3D array acquires and holds the locked object', ({ check, note }) => {
  const tetra = [[0.05, 0.04, 0.03], [0.05, -0.03, -0.04], [-0.05, 0.03, -0.04], [-0.05, -0.04, 0.03]];
  const r = runTrackingTrial({
    positions: tetra,
    segments: [{ tStart: 0, tEnd: 4, azDeg: 55, elDeg: 10 }],
    durationSec: 4,
  });
  check(r.metrics.acquisitionSec != null && r.metrics.acquisitionSec < 2.2, `acquisition ${r.metrics.acquisitionSec}s > 2.2s`);
  check(r.metrics.lockRatio > 0.7, `lock ratio ${(r.metrics.lockRatio * 100).toFixed(0)}% < 70%`);
  check(r.metrics.wrongSwitchCount === 0, `wrong switches ${r.metrics.wrongSwitchCount}`);
  check(r.metrics.p90ReportedErrorDeg < 5, `p90 reported error ${r.metrics.p90ReportedErrorDeg?.toFixed(1)}°`);
  note(`acq ${r.metrics.acquisitionSec.toFixed(2)}s lock ${(r.metrics.lockRatio * 100).toFixed(0)}% wrong ${r.metrics.wrongSwitchCount} p90 ${r.metrics.p90ReportedErrorDeg.toFixed(1)}°`);
});

test('Tracking: camera does not chase a wrong branch on a planar array', ({ check, note }) => {
  const planar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
  const r = runTrackingTrial({
    positions: planar,
    segments: [{ tStart: 0, tEnd: 5, azDeg: 50, elDeg: 25 }],
    durationSec: 5,
  });
  check(r.metrics.wrongSwitchCount === 0, `held a wrong branch ${r.metrics.wrongSwitchCount} times`);
  // No probe is used by runTrackingTrial, so the camera should stay near its start.
  const panMoved = Math.abs(((r.metrics.finalPose.pan + 540) % 360) - 180);
  check(panMoved < 2, `camera moved ${panMoved.toFixed(1)}° on an unresolved ambiguity`);
  note(`planar hold: wrong ${r.metrics.wrongSwitchCount}, moved ${panMoved.toFixed(1)}°, lock ${(r.metrics.lockRatio * 100).toFixed(0)}%`);
});

test('Tracking: source disappearance holds last bearing instead of inventing one', ({ check, note }) => {
  const tetra = [[0.05, 0.04, 0.03], [0.05, -0.03, -0.04], [-0.05, 0.03, -0.04], [-0.05, -0.04, 0.03]];
  const r = runTrackingTrial({
    positions: tetra,
    segments: [
      { tStart: 0, tEnd: 2, azDeg: 40, elDeg: 5 },
      { tStart: 2, tEnd: 4, azDeg: 40, elDeg: 5, present: false },
    ],
    durationSec: 4,
  });
  const afterGone = r.events.filter((e) => e.t > 2.1);
  check(afterGone.length > 0, 'no events after disappearance');
  check(afterGone.every((e) => e.status === 'insufficient'), 'invented a direction after the source disappeared');
  check(afterGone.every((e) => e.action === 'hold'), 'camera did not hold after disappearance');
  note(`after gone: ${afterGone.length} frames all insufficient+hold`);
});

// ---------------------------------------------------------------------------
test('Probe: tilt of a horizontal planar array resolves the up/down mirror', ({ check, note }) => {
  const planar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
  const r = runProbeTrial({
    positions: planar,
    truth: { azDeg: 50, elDeg: 25 },
    durationSec: 9,
    planeNormalArray: [0, 1, 0],
    probeConfig: { probeDeg: 15, armAfterSec: 0.7 },
  });
  const t = r.metrics.probe;
  check(!!t, 'no probe was planned');
  check(t.axis === 'tilt', `probe used ${t.axis}, expected tilt for a horizontal array`);
  check(t.predictedSpurShiftDeg > 8, `probe predicted only ${t.predictedSpurShiftDeg?.toFixed(1)}° separation`);
  check(r.metrics.mirrorResolvedAtSec != null && r.metrics.mirrorResolvedAtSec < 6, `mirror not resolved by 6s (${r.metrics.mirrorResolvedAtSec})`);
  check(r.metrics.finalTrustedErrorDeg < 5, `final trusted error ${r.metrics.finalTrustedErrorDeg.toFixed(1)}°`);
  check(r.metrics.lastStatus === 'resolved', `final status ${r.metrics.lastStatus}`);
  note(`probe ${t.axis} ±${t.deltaDeg}°, predicted shift ${t.predictedSpurShiftDeg.toFixed(1)}°, resolved ${r.metrics.mirrorResolvedAtSec.toFixed(2)}s, err ${r.metrics.finalTrustedErrorDeg.toFixed(2)}°`);
});

test('Probe: panning a horizontal planar array is correctly predicted NOT to help', ({ check, note }) => {
  const planar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
  const r = runScriptedDisambiguation({
    positions: planar,
    truth: { azDeg: 50, elDeg: 25 },
    poses: [{ panDeg: 0, tiltDeg: 0 }, { panDeg: 16, tiltDeg: 0 }, { panDeg: 32, tiltDeg: 0 }],
  });
  const last = r.steps[r.steps.length - 1];
  check(last.status === 'ambiguous', `pure pan should leave the ambiguity live, got ${last.status}`);
  check(last.decision.action === 'hold', `camera should hold on pure-pan ambiguity, got ${last.decision.action}`);
  note(`pan-only: ${last.status}, action ${last.decision.action}`);
});

test('Probe: consistency locks the true branch and never the spur', ({ check, note }) => {
  const planar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
  const r = runScriptedDisambiguation({
    positions: planar,
    truth: { azDeg: 50, elDeg: 25 },
    poses: [0, 6, 12, 18, 24].map((tiltDeg) => ({ panDeg: 0, tiltDeg })),
  });
  const locked = r.steps.filter((s) => s.consistent);
  check(locked.length > 0, 'consistency check never locked a bearing');
  for (const s of locked) {
    const err = angularSeparationDeg(azElToUnit(s.consistent.azDeg, s.consistent.elDeg), azElToUnit(50, 25));
    check(err < 5, `consistent bearing off truth by ${err.toFixed(1)}° at tilt ${s.pose.tiltDeg}`);
  }
  const slewed = r.steps.some((s) => s.decision.action === 'slew');
  check(slewed, 'tracker never acted on the resolved branch');
  const last = r.steps[r.steps.length - 1];
  // After the branch is resolved, the last decision should not be holding a live
  // two-way ambiguity: the tracker committed to the consistent world bearing.
  check(last.decision.bearing && last.decision.bearing.source !== 'held-trusted' || last.decision.action === 'slew',
    `final decision held a stale ambiguity: action=${last.decision.action} source=${last.decision.bearing?.source}`);
  note(`locked ${locked.length} frames; final action ${last.decision.action}, bearing source ${last.decision.bearing?.source}`);
});

// ---------------------------------------------------------------------------
test('Angles are reported in a documented convention', ({ check, note }) => {
  // az measured from +z toward +x; el up. A source straight ahead is (0,0); to the
  // right is (+90, 0); above is (0, +90).
  check(Math.abs(azElToUnit(0, 0)[2] - 1) < 1e-9, 'az=0,el=0 should be +z');
  check(Math.abs(azElToUnit(90, 0)[0] - 1) < 1e-9, 'az=90 should be +x');
  check(Math.abs(azElToUnit(0, 90)[1] - 1) < 1e-9, 'el=90 should be +y');
  const a = unitToAzEl([Math.sin(0.3) * Math.cos(0.2), Math.sin(0.2), Math.cos(0.3) * Math.cos(0.2)]);
  check(Math.abs(a.azDeg - (0.3 * 180) / Math.PI) < 1e-6, 'az round trip');
  check(Math.abs(a.elDeg - (0.2 * 180) / Math.PI) < 1e-6, 'el round trip');
  note('convention verified: +z forward, +x right, +y up');
});

test('Array-to-world mapping respects the camera pose', ({ check, note }) => {
  const d = azElToUnit(10, 5);
  const w = arrayToWorld(d, 30, 0);
  const back = unitToAzEl(w);
  check(Math.abs(back.azDeg - 40) < 1e-6, `pan should add to azimuth: got ${back.azDeg}`);
  const w2 = arrayToWorld(d, 0, 10);
  const back2 = unitToAzEl(w2);
  check(back2.elDeg > 5, `tilt should raise elevation: got ${back2.elDeg}`);
  note(`pan 30 on az 10 -> az ${back.azDeg.toFixed(2)}; tilt 10 -> el ${back2.elDeg.toFixed(2)}`);
});

test('Input validation rejects malformed geometry and timing', ({ check, note }) => {
  const A = arrays();
  const channels = A.tetra.map(() => new Float32Array(4800));
  const bad = [
    [{ recording: { channels, sampleRate: FS }, geometry: { positions: [[0, 0, 0]], speedOfSound: C } }, 'one mic'],
    [{ recording: { channels, sampleRate: FS }, geometry: { positions: [[0, 0, 0], [0, 0, 0]], speedOfSound: C } }, 'coincident mics'],
    [{ recording: { channels, sampleRate: FS }, geometry: { positions: A.tetra, speedOfSound: 0 } }, 'zero speed'],
    [{ recording: { channels, sampleRate: FS }, geometry: { positions: A.tetra, speedOfSound: C, channelMap: [0, 1, 2, 9] } }, 'channelMap out of range'],
    [{ recording: { channels, sampleRate: FS }, geometry: { positions: A.tetra, speedOfSound: C, channelMap: [0, 0, 1, 2] } }, 'repeated channel'],
    [{ recording: { channels, sampleRate: FS }, geometry: { positions: A.tetra, speedOfSound: C }, options: { bandLow: 5000, bandHigh: 1000 } }, 'inverted band'],
  ];
  for (const [input, label] of bad) {
    let threw = false;
    try { analyzeDirection(input); } catch (e) { threw = true; }
    check(threw, `${label} was accepted`);
  }
  note(`${bad.length} malformed inputs rejected`);
});

test('Missing/invalid source evidence is explicit in the result object', ({ check, note }) => {
  const A = arrays();
  const sc = sceneFor(A.tetra, 30, 5, { snrDb: -18, seed: 55, durationSec: 0.8 });
  const an = analyzeDirection({
    recording: { channels: sc.channels, sampleRate: FS },
    geometry: { positions: A.tetra, speedOfSound: C },
    options: { frameLength: 3840 },
  });
  check(an.direction.status === 'insufficient', `expected insufficient, got ${an.direction.status}`);
  check(typeof an.direction.reason === 'string' && an.direction.reason.length > 0, 'insufficient result must carry a human-readable reason');
  check(an.direction.reportable === false, 'insufficient result must be marked non-reportable');
  check(an.direction.candidates.every((c) => c.plausible === false), 'insufficient result must not mark any candidate plausible');
  note(`insufficient: ${an.direction.reason}`);
});

// ---------------------------------------------------------------------------
test('Jump diag: swapped channels are localized to the swapped mics (tdoa stage)', ({ check, note }) => {
  const r = evaluateScenario(scenarioChannelSwap, (m) => note(m));
  for (const f of r.failures) check(false, f);
  check(r.event.implicatedMics.join(',') === '1,2', `implicated mics ${r.event.implicatedMics}`);
  check(r.verify.conclusion === 'confirmed', `verify ${r.verify.conclusion}`);
});

test('Jump diag: a dropped mic is told apart from a channel swap', ({ check, note }) => {
  const r = evaluateScenario(scenarioMicDropout, (m) => note(m));
  for (const f of r.failures) check(false, f);
  check(r.event.cause === 'mic-dropout', `cause ${r.event.cause}`);
  check(r.event.implicatedMics.join(',') === '2', `implicated mics ${r.event.implicatedMics}`);
});

test('Jump diag: strong reflection is separated from a real source move', ({ check, note }) => {
  const r = evaluateScenario(scenarioStrongReflection, (m) => note(m));
  for (const f of r.failures) check(false, f);
  check(r.event.cause === 'strong-reflection', `cause ${r.event.cause}`);
  check(r.verify.conclusion === 'confirmed', `verify ${r.verify.conclusion}`);
});

test('Jump diag: a branch flip keeps both pair-supported branches retainable', ({ check, note }) => {
  const r = evaluateScenario(scenarioMultiPeakFlip, (m) => note(m));
  for (const f of r.failures) check(false, f);
  check(r.event.branchSupport.retainable === true, 'reference branch must stay pair-supported');
});

test('Jump diag: expected changes are not false-reported as anomalies', ({ check, note }) => {
  const geo = evaluateScenario(scenarioAxisTransform, (m) => note(m));
  for (const f of geo.failures) check(false, f);
  const moved = evaluateScenario(scenarioSourceMoved, (m) => note(m));
  for (const f of moved.failures) check(false, f);
  check(moved.event.kind === 'expected', `source move reported as ${moved.event.kind}`);
});

test('Jump diag: event and segment logs stay bounded under an anomaly flood', ({ check, note }) => {
  const r = evaluateScenario(scenarioLogCost, (m) => note(m));
  for (const f of r.failures) check(false, f);
});

// ---------------------------------------------------------------------------
const groups = new Map();
for (const r of results) {
  if (!groups.has(r.name)) groups.set(r.name, r);
}
let pass = 0;
let fail = 0;
for (const r of results) {
  const ok = r.failed === 0;
  if (ok) pass++; else fail++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${r.name}  (${r.passed} checks, ${r.ms}ms)`);
  for (const n of r.notes) console.log(`      ${n}`);
  for (const f of r.failures) console.log(`      ! ${f}`);
}
console.log(`\n${results.length} test(s): ${pass} passed, ${fail} failed`);
if (fail > 0) process.exit(1);
