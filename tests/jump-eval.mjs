// Direction-jump diagnostics: scenario evaluation.
//
// Each scenario feeds real synthesized multichannel audio through the real
// analyzeDirection() pipeline and the DirectionJumpDiagnostics watcher, then
// checks that the diagnosed cause, stage and implicated channel pairs match
// the injected condition. Imported by tests/run-tests.mjs (kernel tests) and
// runnable directly as a CLI assistant:
//
//   node tests/jump-eval.mjs [--verbose]
//
// Scenarios:
//   channel-swap        two recording channels exchanged, map left stale
//   mic-dropout         one channel goes silent mid-stream
//   strong-reflection   direct source plus a stronger coherent echo
//   multi-peak-flip     solver branch ranking flips between pair-supported
//                       mirror candidates (TDOAs unchanged)
//   sign-error          one pair reports the negated lag (crafted frame)
//   axis-transform      geometry/pose changes must not be false-reported
//   source-moved        a real source move must not be false-reported
//   track-switch        decision bearing leaves the pair-supported reference
//   log-cost            event/segment rings stay bounded under an event flood

import { renderScene } from '../src/core/synthesize.js';
import { analyzeDirection } from '../src/core/analysis.js';
import { DirectionJumpDiagnostics } from '../src/core/jumpdiag.js';
import { azElToUnit, angularSeparationDeg } from '../src/core/geometry.js';
import { pathToFileURL } from 'node:url';

const FS = 48000;
const C = 343;
const TETRA = [[0.05, 0.04, 0.03], [0.05, -0.03, -0.04], [-0.05, 0.03, -0.04], [-0.05, -0.04, 0.03]];
const SQUARE = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];

function scene(positions, az, el, opts = {}) {
  return renderScene({
    positions,
    source: { azDeg: az, elDeg: el },
    sampleRate: FS,
    speedOfSound: C,
    durationSec: opts.durationSec ?? 1.0,
    snrDb: opts.snrDb ?? 28,
    seed: opts.seed ?? 11,
    bandLow: 300,
    bandHigh: 6000,
    equalPower: true,
  });
}

function analyze(channels, positions, opts = {}) {
  return analyzeDirection({
    recording: { channels, sampleRate: FS, startSample: opts.startSample ?? 4800 },
    geometry: {
      positions,
      speedOfSound: C,
      referenceIndex: 0,
      channelMap: opts.channelMap,
    },
    options: {
      frameLength: opts.frameLength ?? 9600,
      model: 'auto',
      bandLow: 300,
      bandHigh: 6000,
    },
  });
}

const rec = (channels) => ({ channels, sampleRate: FS });

/** Two clean frames to establish the reference, then the fault frame. */
function watch(diag, analysis, recording, extra = {}) {
  return diag.observe({ analysis, pose: {}, recording, ...extra });
}

// ---------------------------------------------------------------------------
export function scenarioChannelSwap() {
  const sc = scene(TETRA, 70, 15);
  const diag = new DirectionJumpDiagnostics();
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  const swapped = sc.channels.slice();
  [swapped[1], swapped[2]] = [swapped[2], swapped[1]];
  const ev = watch(diag, analyze(swapped, TETRA), rec(swapped));
  const v = ev ? diag.verify(ev.id) : null;
  return {
    name: 'channel-swap',
    event: ev,
    verify: v,
    expect: {
      kind: 'anomaly', cause: 'channel-swap', stage: 'tdoa', mics: [1, 2],
      conclusion: 'confirmed',
    },
  };
}

export function scenarioMicDropout() {
  const sc = scene(TETRA, 70, 15);
  const diag = new DirectionJumpDiagnostics();
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  const dropped = sc.channels.map((ch, i) => (i === 2 ? new Float32Array(ch.length) : ch));
  const ev = watch(diag, analyze(dropped, TETRA), rec(dropped));
  const v = ev ? diag.verify(ev.id) : null;
  return {
    name: 'mic-dropout',
    event: ev,
    verify: v,
    expect: {
      kind: 'anomaly', cause: 'mic-dropout', stage: 'tdoa', mics: [2],
      conclusion: 'confirmed',
    },
  };
}

export function scenarioStrongReflection() {
  // Controlled reflection: the direct source plus the SAME waveform arriving
  // from the wall direction, stronger than the direct path and delayed by a
  // bulk path difference that pushes the cross terms out of the TDOA window.
  const direct = scene(TETRA, 60, 10, { seed: 21, snrDb: 35 });
  const echo = scene(TETRA, 210, -20, { seed: 21, snrDb: 35 });
  const diag = new DirectionJumpDiagnostics();
  watch(diag, analyze(direct.channels, TETRA), rec(direct.channels));
  watch(diag, analyze(direct.channels, TETRA), rec(direct.channels));
  const bulkShift = Math.round(0.003 * FS);
  const gain = 1.05;
  const mixed = direct.channels.map((ch, m) => {
    const out = new Float32Array(ch.length);
    for (let i = 0; i < ch.length; i++) {
      const e = i - bulkShift >= 0 ? echo.channels[m][i - bulkShift] : 0;
      out[i] = ch[i] + gain * e;
    }
    return out;
  });
  const ev = watch(diag, analyze(mixed, TETRA), rec(mixed));
  const v = ev ? diag.verify(ev.id) : null;
  return {
    name: 'strong-reflection',
    event: ev,
    verify: v,
    expect: { kind: 'anomaly', cause: 'strong-reflection', stage: 'tdoa', conclusion: 'confirmed' },
  };
}

export function scenarioMultiPeakFlip() {
  const sc = scene(SQUARE, 60, 35, { snrDb: 30 });
  const diag = new DirectionJumpDiagnostics();
  const a1 = analyze(sc.channels, SQUARE);
  watch(diag, a1, rec(sc.channels));
  watch(diag, a1, rec(sc.channels));
  // Craft the solver branch flip: same measurements, mirror ranked first.
  const best = a1.direction.best;
  const other = a1.direction.candidates.find(
    (c) => c.plausible
      && angularSeparationDeg(azElToUnit(c.azDeg, c.elDeg), azElToUnit(best.azDeg, best.elDeg)) > 30,
  );
  if (!other) return { name: 'multi-peak-flip', event: null, skipped: 'no mirror candidate', expect: {} };
  const flipped = {
    ...a1,
    direction: {
      ...a1.direction,
      best: other,
      candidates: [other, ...a1.direction.candidates.filter((c) => c !== other)],
    },
  };
  const ev = watch(diag, flipped, rec(sc.channels));
  const v = ev ? diag.verify(ev.id) : null;
  return {
    name: 'multi-peak-flip',
    event: ev,
    verify: v,
    expect: {
      kind: 'anomaly', cause: 'multi-peak-ambiguity', stage: 'solve',
      retainable: true, conclusion: 'confirmed',
    },
  };
}

export function scenarioSignError() {
  const sc = scene(TETRA, 50, 20);
  const diag = new DirectionJumpDiagnostics();
  const a1 = analyze(sc.channels, TETRA);
  watch(diag, a1, rec(sc.channels));
  watch(diag, a1, rec(sc.channels));
  // Craft a software sign fault: one pair reports the negated lag.
  const pairs = a1.tdoa.pairs.map((p) => ({ ...p }));
  let k = 0;
  pairs.forEach((p, i) => { if (Math.abs(p.lagSec) > Math.abs(pairs[k].lagSec)) k = i; });
  const target = pairs[k].pair.slice();
  pairs[k] = { ...pairs[k], lagSec: -pairs[k].lagSec, lagSamples: -pairs[k].lagSamples };
  const best = a1.direction.best;
  const crafted = {
    ...a1,
    tdoa: { ...a1.tdoa, pairs },
    direction: {
      ...a1.direction,
      best: { ...best, azDeg: (best.azDeg + 45) % 360 },
    },
  };
  const ev = watch(diag, crafted, rec(sc.channels));
  const v = ev ? diag.verify(ev.id) : null;
  return {
    name: 'sign-error',
    event: ev,
    verify: v,
    targetPair: target,
    expect: { kind: 'anomaly', cause: 'sign-error', stage: 'tdoa', pair: target, conclusion: 'confirmed' },
  };
}

export function scenarioAxisTransform() {
  const sc = scene(TETRA, 70, 15);
  const diag = new DirectionJumpDiagnostics();
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  // Geometry edit (scaled layout) with the operator note: expected, not anomaly.
  const bigger = TETRA.map((p) => p.map((v) => v * 1.6));
  const evGeo = watch(diag, analyze(sc.channels, bigger), rec(sc.channels), { note: { geometryChanged: true } });
  // Camera pan with the world source fixed: no event at all.
  const scPan = scene(TETRA, 40, 15); // array-frame 40 deg == world 70 deg after pan +30
  const evPan = watch(diag, analyze(scPan.channels, TETRA), rec(scPan.channels), { pose: { panDeg: 30, tiltDeg: 0 } });
  return {
    name: 'axis-transform',
    event: evGeo,
    panEvent: evPan,
    expect: { kind: 'expected', cause: 'axis-transform', panSilent: true },
  };
}

export function scenarioSourceMoved() {
  const sc = scene(TETRA, 70, 15);
  const diag = new DirectionJumpDiagnostics();
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  const moved = scene(TETRA, 130, 25);
  const ev = watch(diag, analyze(moved.channels, TETRA), rec(moved.channels));
  return {
    name: 'source-moved',
    event: ev,
    expect: { kind: 'expected', cause: 'source-moved' },
  };
}

export function scenarioTrackSwitch() {
  const sc = scene(TETRA, 45, 10);
  const diag = new DirectionJumpDiagnostics();
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  const a = analyze(sc.channels, TETRA);
  watch(diag, a, rec(sc.channels));
  // The solver stays on the reference but the decision bearing leaves it.
  const ev = watch(diag, a, rec(sc.channels), {
    decision: { bearing: { world: azElToUnit(220, 0), source: 'tracked' } },
  });
  return {
    name: 'track-switch',
    event: ev,
    expect: { kind: 'anomaly', cause: 'track-switch', stage: 'track' },
  };
}

export function scenarioLogCost() {
  const sc = scene(TETRA, 70, 15);
  const diag = new DirectionJumpDiagnostics({ maxEvents: 6, maxSegments: 2, mergeFrameGap: 0 });
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  watch(diag, analyze(sc.channels, TETRA), rec(sc.channels));
  const swapped = sc.channels.slice();
  [swapped[1], swapped[2]] = [swapped[2], swapped[1]];
  for (let i = 0; i < 12; i++) watch(diag, analyze(swapped, TETRA), rec(swapped));
  const stats = diag.stats();
  return {
    name: 'log-cost',
    event: null,
    stats,
    expect: { maxEvents: 6, maxSegments: 2 },
  };
}

export const SCENARIOS = [
  scenarioChannelSwap,
  scenarioMicDropout,
  scenarioStrongReflection,
  scenarioMultiPeakFlip,
  scenarioSignError,
  scenarioAxisTransform,
  scenarioSourceMoved,
  scenarioTrackSwitch,
  scenarioLogCost,
];

function fmtEvent(ev) {
  if (!ev) return '<no event>';
  const mics = ev.implicatedMics?.length ? ` M${ev.implicatedMics.join('/M')}` : '';
  return `#${ev.id} ${ev.kind}/${ev.cause} stage=${ev.stage} jump=${ev.jumpDeg ?? '—'}°${mics}`;
}

export function evaluateScenario(fn, note = () => {}) {
  const r = fn();
  const failures = [];
  const ok = (cond, msg) => { if (!cond) failures.push(msg); return cond; };
  const ev = r.event;
  const x = r.expect;

  if (r.name === 'log-cost') {
    const s = r.stats;
    ok(s.events <= x.maxEvents, `events ${s.events} exceed cap ${x.maxEvents}`);
    ok(s.segmentsHeld <= x.maxSegments, `segments ${s.segmentsHeld} exceed cap ${x.maxSegments}`);
    ok(s.droppedEvents > 0, 'flood should have dropped events');
    note(`events ${s.events}/${s.maxEvents} segments ${s.segmentsHeld}/${s.maxSegments} dropped ${s.droppedEvents}+${s.droppedSegments}`);
    return { name: r.name, failures };
  }
  if (r.name === 'axis-transform') {
    ok(ev && ev.kind === x.kind && ev.cause === x.cause, `geometry edit -> ${fmtEvent(ev)}, expected ${x.kind}/${x.cause}`);
    ok(!r.panEvent || r.panEvent.kind !== 'anomaly', `camera pan produced ${fmtEvent(r.panEvent)} (must stay silent)`);
    note(`geometry edit -> ${fmtEvent(ev)}; pan -> ${r.panEvent ? fmtEvent(r.panEvent) : 'silent'}`);
    return { name: r.name, failures };
  }
  if (!ok(!!ev, 'no event was raised')) return { name: r.name, failures };
  note(fmtEvent(ev));
  if (x.kind) ok(ev.kind === x.kind, `kind ${ev.kind} != ${x.kind}`);
  if (x.cause) ok(ev.cause === x.cause, `cause ${ev.cause} != ${x.cause}`);
  if (x.stage) ok(ev.stage === x.stage, `stage ${ev.stage} != ${x.stage}`);
  if (x.mics) ok(ev.implicatedMics.join(',') === x.mics.join(','), `mics [${ev.implicatedMics}] != [${x.mics}]`);
  if (x.pair) {
    const hit = ev.implicatedPairs.some((p) => p[0] === x.pair[0] && p[1] === x.pair[1]);
    ok(hit, `pair ${x.pair} not in implicated ${JSON.stringify(ev.implicatedPairs)}`);
  }
  if (x.retainable != null) {
    ok(ev.branchSupport.retainable === x.retainable, `retainable ${ev.branchSupport.retainable} != ${x.retainable}`);
    ok(ev.branchSupport.reference.frac >= 0.6, `reference branch support ${ev.branchSupport.reference?.frac}`);
    ok(ev.branchSupport.reported.frac >= 0.6, `reported branch support ${ev.branchSupport.reported?.frac}`);
  }
  if (x.conclusion) {
    ok(!!r.verify, 'verify() returned nothing');
    if (r.verify) {
      ok(r.verify.conclusion === x.conclusion, `verify conclusion ${r.verify.conclusion} != ${x.conclusion}`);
      note(`verify: ${r.verify.steps.map((s) => `${s.step}${s.ok == null ? '' : s.ok ? '✓' : '✗'}`).join(' -> ')} => ${r.verify.conclusion}`);
    }
  }
  return { name: r.name, failures, event: ev, verify: r.verify };
}

const isMain = process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href;
if (isMain) {
  const verbose = process.argv.includes('--verbose');
  let failed = 0;
  for (const fn of SCENARIOS) {
    const notes = [];
    const r = evaluateScenario(fn, (msg) => notes.push(msg));
    const ok = r.failures.length === 0;
    if (!ok) failed++;
    console.log(`${ok ? 'PASS' : 'FAIL'}  ${r.name}`);
    if (verbose || !ok) for (const n of notes) console.log(`      ${n}`);
    for (const f of r.failures) console.log(`      ! ${f}`);
  }
  console.log(`\n${SCENARIOS.length} scenario(s): ${SCENARIOS.length - failed} passed, ${failed} failed`);
  process.exit(failed ? 1 : 0);
}
