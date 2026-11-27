// Direction-jump diagnostics: array direction tracking and diagnosis.
//
// Watches the per-frame analysis output (cross-channel TDOAs, GCC candidate
// peaks, array coordinates) and answers one question: when the reported
// direction jumps while the measured TDOAs say the source did not move, which
// stage went wrong, and which channel pair carries the erroneous delay?
//
// Reference: the tracker's DEFERRED attribution (last trusted world bearing)
// is the stability reference. A jump is the solver's best world direction
// leaving that reference while the pairs still support it. Branches that
// remain supported by mic pairs are explicitly kept (branchSupport), never
// smoothed away; raw multichannel segments are retained (capped) so every
// conclusion can be re-run against the original samples.
//
// Causes are separated by observation, not by smoothing:
//   - mic-dropout:        one microphone's pairs lose coherence and its channel
//                         power collapses while the rest stay healthy;
//   - channel-swap:       a transposition of two microphones makes the measured
//                         TDOAs consistent with the reference direction again;
//   - sign-error:         a pair reports tau ~= -tau_predicted and the value is
//                         not even a peak in its own GCC curve;
//   - strong-reflection:  pairs keep a direct-path-consistent GCC peak but a
//                         stronger secondary peak wins, and the jumped TDOAs
//                         fit one coherent second direction (wall echo or a
//                         second coherent source);
//   - multi-peak-ambiguity: the TDOAs did not move; the solver's branch ranking
//                         flipped between pair-supported candidates;
//   - track-switch:       the solver stayed on the reference but the decision
//                         bearing left it without pair support;
//   - axis-transform / source-moved / source-gone: expected, not anomalies.
//
// Log cost is capped: frame summaries, events and retained raw segments all
// live in fixed-size rings; stats() reports what was dropped.

import {
  azElToUnit,
  unitToAzEl,
  angularSeparationDeg,
  predictedOffsets,
  arrayToWorld,
  worldToArray,
  norm,
} from './geometry.js';
import { solveDirection } from './solver.js';
import { analyzeDirection } from './analysis.js';

export const JUMP_DIAG_DEFAULTS = Object.freeze({
  jumpDeg: 12,               // best-direction swing that counts as a jump
  tauSigma: 3,               // a pair supports a direction within this many sigma
  tauFloorSec: 1e-5,         // ...or at least this absolute tolerance
  minConsistentFrac: 0.6,    // fraction of pairs that makes a branch "supported"
  tauChangeSigma: 3,         // per-pair frame-to-frame change detection
  tauChangeFloorSec: 1.2e-5,
  dropoutCoherence: 0.08,    // a pair is "dead" below this coherence...
  dropoutPeakToFloor: 1.15,  // ...or this peak/floor ratio
  dropoutPowerRatio: 0.2,    // dropped mic power vs the array median
  swapMinFrac: 0.75,         // a transposition must explain this many pairs
  swapMargin: 0.34,          // ...and beat the identity mapping by this much
  swapMaxMics: 8,
  reflectionMinDirectPeaks: 0.67, // fraction of jumped pairs that still show the direct peak
  reflectionDirectLevel: 1.2,  // curve level at the direct lag vs the floor
  reflectionPeakProminence: 1.5, // min peak/floor for a GCC peak to count
  directPeakMatchSec: 3e-5,    // verify(): window to re-find the (echo-biased) direct peak
  maxEvents: 24,             // event ring size (log cost cap)
  maxSegments: 4,            // retained raw multichannel segments (log cost cap)
  frameSummaryKeep: 60,      // compact per-frame trail
  mergeFrameGap: 3,          // same-cause events within this gap merge into one
  verifyTolDeg: 6,           // re-run verification angular tolerance
  peakCount: 4,              // GCC candidate peaks kept per implicated pair
});

/** GCC curve level at a specific lag, relative to the curve floor. */
export function curveLevelAt(pair, lagSec, sampleRate) {
  const { lags, values } = pair;
  if (!lags || !values || !sampleRate) return 0;
  let sq = 0;
  for (let t = 0; t < values.length; t++) sq += values[t] * values[t];
  const floor = Math.sqrt(sq / Math.max(1, values.length)) + 1e-12;
  const lagSamples = Math.round(lagSec * sampleRate);
  let best = -Infinity;
  for (let t = 0; t < lags.length; t++) {
    if (Math.abs(lags[t] - lagSamples) <= 1 && values[t] > best) best = values[t];
  }
  return best === -Infinity ? 0 : best / floor;
}

/** Local maxima of a stored GCC curve, strongest first. */
export function extractPeaks(pair, opts = {}) {
  const sampleRate = opts.sampleRate ?? pair.sampleRate;
  const maxPeaks = opts.maxPeaks ?? JUMP_DIAG_DEFAULTS.peakCount;
  const minProminence = opts.minProminence ?? JUMP_DIAG_DEFAULTS.reflectionPeakProminence;
  const { lags, values } = pair;
  if (!lags || !values || !sampleRate) return [];
  let sq = 0;
  for (let t = 0; t < values.length; t++) sq += values[t] * values[t];
  const floor = Math.sqrt(sq / Math.max(1, values.length)) + 1e-12;
  const peaks = [];
  for (let t = 1; t < values.length - 1; t++) {
    const v = values[t];
    if (v < values[t - 1] || v <= values[t + 1]) continue;
    const prominence = v / floor;
    if (prominence < minProminence) continue;
    peaks.push({ lagSamples: lags[t], lagSec: lags[t] / sampleRate, value: v, prominence });
  }
  peaks.sort((a, b) => b.value - a.value);
  return peaks.slice(0, maxPeaks);
}

/**
 * Which mic pairs support a candidate direction? This is the "retain the
 * pair-supported branch" check: a branch stays plausible exactly while enough
 * pairs agree with it within their uncertainty.
 * @returns {{consistentFrac:number, count:number, total:number, chi2:number, perPair:object[]}}
 */
export function branchSupport(o) {
  const {
    pairs,
    positions,
    speedOfSound,
    referenceIndex = 0,
    directionWorld,
    pose = {},
    model = 'far',
    distanceM = 1.5,
    tauSigma = JUMP_DIAG_DEFAULTS.tauSigma,
    tauFloorSec = JUMP_DIAG_DEFAULTS.tauFloorSec,
  } = o;
  const dArr = worldToArray(directionWorld, pose.panDeg ?? 0, pose.tiltDeg ?? 0);
  const offs = predictedOffsets({
    positions,
    referenceIndex,
    direction: dArr,
    model,
    distanceM,
    speedOfSound,
  });
  const perPair = pairs.map((p) => {
    const tau = p.lagSec ?? p.tau;
    const sigma = p.sigmaTauSec ?? p.sigma ?? 1e-6;
    const pred = offs[p.pair[0]] - offs[p.pair[1]];
    const tol = Math.max(tauSigma * sigma, tauFloorSec);
    const err = tau - pred;
    return {
      pair: p.pair,
      tauMeasSec: tau,
      tauPredSec: pred,
      errSec: err,
      z: sigma > 0 ? err / sigma : 0,
      tolSec: tol,
      consistent: Math.abs(err) <= tol,
    };
  });
  const count = perPair.filter((x) => x.consistent).length;
  const chi2 = perPair.reduce((s, x) => s + x.z * x.z, 0);
  return {
    consistentFrac: perPair.length ? count / perPair.length : 0,
    count,
    total: perPair.length,
    chi2,
    perPair,
  };
}

function pairKeyOf(pair) {
  return `${pair[0]}-${pair[1]}`;
}

function meanPower(ch, start, end) {
  const s = Math.max(0, Math.min(start, ch.length));
  const e = Math.max(s, Math.min(end, ch.length));
  if (e - s < 16) return 0;
  const step = Math.max(1, Math.floor((e - s) / 2048)); // cap the per-frame cost
  let acc = 0;
  let n = 0;
  for (let i = s; i < e; i += step) { acc += ch[i] * ch[i]; n++; }
  return n ? acc / n : 0;
}

/** Per-microphone window powers, in mic order, from the raw recording. */
export function channelPowers(recording, metadata) {
  return metadata.channelMap.map((recCh) => {
    const ch = recording.channels[recCh];
    if (!ch) return 0;
    return meanPower(ch, metadata.startSample ?? 0, (metadata.startSample ?? 0) + (metadata.frameLength ?? ch.length));
  });
}

/** One microphone whose pairs all died while the others stayed healthy. */
export function scanDropout(pairs, powers, nMics, cfg = JUMP_DIAG_DEFAULTS) {
  if (nMics < 3) return null;
  const dead = [];
  for (let m = 0; m < nMics; m++) {
    const pm = pairs.filter((p) => p.pair[0] === m || p.pair[1] === m);
    dead.push(pm.length > 0 && pm.every(
      (p) => (p.coherence ?? 1) < cfg.dropoutCoherence || (p.peakToFloor ?? 2) < cfg.dropoutPeakToFloor,
    ));
  }
  const deadMics = dead.map((d, i) => (d ? i : -1)).filter((i) => i >= 0);
  const healthy = dead.map((d, i) => (!d ? i : -1)).filter((i) => i >= 0);
  if (deadMics.length !== 1 || healthy.length < 2) return null;
  const mic = deadMics[0];
  if (powers) {
    const sorted = [...powers].sort((a, b) => a - b);
    const median = sorted[Math.floor(sorted.length / 2)] || 0;
    // Coherence collapse with normal channel power is interference, not a dropout.
    if (!(powers[mic] < cfg.dropoutPowerRatio * Math.max(median, 1e-12))) return null;
  }
  return {
    mic,
    pairs: pairs.filter((p) => p.pair[0] === mic || p.pair[1] === mic).map((p) => p.pair),
  };
}

/** A transposition of two mics that makes the measured TDOAs fit the reference. */
export function scanSwap(pairs, positions, referenceIndex, referenceWorld, pose, model, distanceM, speedOfSound, cfg = JUMP_DIAG_DEFAULTS) {
  const n = positions.length;
  if (n < 3 || n > cfg.swapMaxMics || !pairs.length) return null;
  const dArr = worldToArray(referenceWorld, pose.panDeg ?? 0, pose.tiltDeg ?? 0);
  const offs = predictedOffsets({ positions, referenceIndex, direction: dArr, model, distanceM, speedOfSound });
  const score = (perm) => {
    let count = 0;
    for (const p of pairs) {
      const pred = offs[perm[p.pair[0]]] - offs[perm[p.pair[1]]];
      const tol = Math.max(cfg.tauSigma * (p.sigmaTauSec ?? 1e-6), cfg.tauFloorSec);
      if (Math.abs(p.lagSec - pred) <= tol) count++;
    }
    return count / pairs.length;
  };
  const identity = [...Array(n).keys()];
  const idFrac = score(identity);
  let best = null;
  for (let a = 0; a < n; a++) {
    for (let b = a + 1; b < n; b++) {
      const perm = identity.slice();
      [perm[a], perm[b]] = [perm[b], perm[a]];
      const frac = score(perm);
      if (!best || frac > best.frac) best = { a, b, frac };
    }
  }
  if (best && best.frac >= cfg.swapMinFrac && best.frac >= idFrac + cfg.swapMargin) {
    return { mics: [best.a, best.b], swappedFrac: best.frac, identityFrac: idFrac };
  }
  return null;
}

/** Pairs whose reported lag is the negated prediction and not a curve peak. */
export function scanSignError(pairs, positions, referenceIndex, referenceWorld, pose, model, distanceM, speedOfSound, sampleRate, cfg = JUMP_DIAG_DEFAULTS) {
  const dArr = worldToArray(referenceWorld, pose.panDeg ?? 0, pose.tiltDeg ?? 0);
  const offs = predictedOffsets({ positions, referenceIndex, direction: dArr, model, distanceM, speedOfSound });
  const flipped = [];
  for (const p of pairs) {
    const pred = offs[p.pair[0]] - offs[p.pair[1]];
    const tol = Math.max(cfg.tauSigma * (p.sigmaTauSec ?? 1e-6), cfg.tauFloorSec);
    if (Math.abs(pred) <= 2 * tol) continue; // degenerate pair axis: sign is untestable
    if (Math.abs(p.lagSec + pred) > tol) continue; // not the negation
    // A real estimator output is a peak of its own GCC curve; a software sign
    // fault is not. The curve check separates sign errors from peak hops.
    const peaks = extractPeaks(p, { sampleRate, maxPeaks: cfg.peakCount, minProminence: cfg.reflectionPeakProminence });
    const isCurvePeak = peaks.some((pk) => Math.abs(pk.lagSec - p.lagSec) <= tol);
    if (!isCurvePeak) flipped.push(p.pair);
  }
  return flipped.length ? flipped : null;
}

/**
 * Strong-reflection check: pairs that left the reference direction still show
 * a direct-path GCC peak, and their jumped TDOAs fit one coherent second
 * direction (a wall echo or a second coherent source).
 */
export function scanReflection(o) {
  const { pairs, positions, referenceIndex, referenceWorld, pose, model, distanceM, speedOfSound, sampleRate } = o;
  const cfg = o.cfg ?? JUMP_DIAG_DEFAULTS;
  const support = branchSupport({
    pairs, positions, referenceIndex, speedOfSound,
    directionWorld: referenceWorld, pose, model, distanceM,
    tauSigma: cfg.tauSigma, tauFloorSec: cfg.tauFloorSec,
  });
  const bad = support.perPair.filter((x) => !x.consistent);
  if (bad.length < 2) return null;
  const byKey = new Map(pairs.map((p) => [pairKeyOf(p.pair), p]));
  // The direct path survives as a curve level at the predicted direct lag.
  // A real source move leaves nothing there (noise level) on most pairs.
  const directAlive = [];
  for (const x of bad) {
    const p = byKey.get(pairKeyOf(x.pair));
    if (!p?.lags) continue;
    if (curveLevelAt(p, x.tauPredSec, sampleRate) >= cfg.reflectionDirectLevel) directAlive.push(x.pair);
  }
  if (directAlive.length < Math.ceil(bad.length * cfg.reflectionMinDirectPeaks)) return null;
  const obs = bad.map((x) => {
    const p = byKey.get(pairKeyOf(x.pair));
    return {
      pair: x.pair,
      tau: x.tauMeasSec,
      sigma: p?.sigmaTauSec ?? 1e-6,
      coherence: p?.coherence ?? 1,
      peakToFloor: p?.peakToFloor ?? 2,
    };
  });
  let fit = null;
  try {
    fit = solveDirection({
      observations: obs,
      positions,
      referenceIndex,
      speedOfSound,
      model: 'far',
      quality: { minCoherence: 0.05, minPeakToFloor: 1.2 },
    });
  } catch {
    return null;
  }
  if (!fit?.best || fit.status === 'insufficient') return null;
  const secondWorld = arrayToWorld(
    azElToUnit(fit.best.azDeg, fit.best.elDeg),
    pose.panDeg ?? 0,
    pose.tiltDeg ?? 0,
  );
  if (angularSeparationDeg(secondWorld, referenceWorld) < cfg.jumpDeg / 2) return null;
  return {
    badPairs: bad.map((x) => x.pair),
    directAlivePairs: directAlive,
    secondWorld,
    secondAzEl: unitToAzEl(secondWorld),
    chi2Red: fit.fit?.chi2Red ?? null,
  };
}

export class DirectionJumpDiagnostics {
  constructor(config = {}) {
    this.config = { ...JUMP_DIAG_DEFAULTS, ...config };
    this._events = [];
    this._segmentOrder = [];
    this._summaries = [];
    this._frame = 0;
    this._nextEventId = 1;
    this._geoKey = null;
    this._lastPairs = null;
    this._lastStatus = null;
    this._referenceWorld = null;
    this._lastDropoutKey = null;
    this._droppedEvents = 0;
    this._droppedSegments = 0;
  }

  /**
   * Feed one analysis frame.
   * @param {object} o
   * @param {object} o.analysis   analyzeDirection() output
   * @param {object} [o.pose]     {panDeg, tiltDeg} camera pose for world mapping
   * @param {object} [o.decision] tracker decision (deferred attribution reference)
   * @param {object} [o.recording] {channels, sampleRate} raw buffers (segment retention)
   * @param {object} [o.note]     {geometryChanged, wiringChanged, sourceChanged}
   * @returns {object|null} the new event, or null (merged/absent)
   */
  observe(o) {
    const cfg = this.config;
    const { analysis, pose = {}, decision = null, recording = null, note = null } = o;
    this._frame++;
    const m = analysis.metadata;
    const dir = analysis.direction;
    const pairs = analysis.tdoa?.pairs ?? [];
    const positions = m.positions;
    const speedOfSound = m.speedOfSound;
    const pan = pose.panDeg ?? 0;
    const tilt = pose.tiltDeg ?? 0;
    const model = m.model ?? 'far';
    const distanceM = dir.nearDistanceM ?? 1.5;

    // --- array coordinates tracking: geometry/map edits invalidate tau deltas
    const geoKey = JSON.stringify([
      positions.map((p) => p.map((v) => Math.round(v * 1e4))),
      m.channelMap,
    ]);
    const geometryChanged = !!note?.geometryChanged
      || (this._geoKey != null && geoKey !== this._geoKey);
    if (this._geoKey != null && geoKey !== this._geoKey) this._lastPairs = null;
    this._geoKey = geoKey;

    const powers = recording ? channelPowers(recording, m) : null;

    const bestWorld = dir.best
      ? arrayToWorld(azElToUnit(dir.best.azDeg, dir.best.elDeg), pan, tilt)
      : null;

    // --- reference: the deferred attribution. A trusted bearing is adopted as
    // the reference only while the pairs actually support it; an unsupported
    // trusted bearing is itself the anomaly (track-switch).
    let trustedWorld = (decision?.bearing?.source === 'held-trusted' || decision?.bearing?.source === 'tracked')
      && decision.bearing.world ? norm(decision.bearing.world) : null;
    let trustedUnsupported = false;
    if (trustedWorld && pairs.length) {
      const supTrusted = branchSupport({
        pairs, positions, referenceIndex: m.referenceIndex, speedOfSound,
        directionWorld: trustedWorld, pose, model, distanceM,
        tauSigma: cfg.tauSigma, tauFloorSec: cfg.tauFloorSec,
      });
      if (supTrusted.consistentFrac >= cfg.minConsistentFrac) {
        this._referenceWorld = trustedWorld;
      } else {
        trustedUnsupported = true;
      }
    } else if (trustedWorld) {
      this._referenceWorld = trustedWorld;
    }
    if (!this._referenceWorld && bestWorld && (dir.status === 'resolved' || dir.status === 'ambiguous')) {
      this._referenceWorld = bestWorld;
    }
    const referenceWorld = this._referenceWorld;

    // --- candidate peaks per pair (compact trail: top-2 per pair)
    const peakBrief = pairs.map((p) => ({
      pair: p.pair,
      top: extractPeaks(p, { sampleRate: m.sampleRate, maxPeaks: 2, minProminence: 1.2 })
        .map((pk) => [Number(pk.lagSec.toExponential(3)), Number(pk.prominence.toFixed(1))]),
    }));

    // --- frame-to-frame tau changes
    const changedPairs = [];
    if (this._lastPairs) {
      const prev = new Map(this._lastPairs.map((p) => [pairKeyOf(p.pair), p]));
      for (const p of pairs) {
        const q = prev.get(pairKeyOf(p.pair));
        if (!q) continue;
        const tol = Math.max(
          cfg.tauChangeSigma * Math.max(p.sigmaTauSec ?? 0, q.sigmaTauSec ?? 0),
          cfg.tauChangeFloorSec,
        );
        if (Math.abs(p.lagSec - q.lagSec) > tol) changedPairs.push(p.pair);
      }
    }

    const jumpDeg = (referenceWorld && bestWorld)
      ? angularSeparationDeg(referenceWorld, bestWorld)
      : null;
    const solverJumped = jumpDeg != null && jumpDeg > cfg.jumpDeg;
    const statusDrop = !!referenceWorld && dir.status === 'insufficient'
      && this._lastStatus != null && this._lastStatus !== 'insufficient';

    const dropout = scanDropout(pairs, powers, positions.length, cfg);
    const dropoutKey = dropout ? `m${dropout.mic}` : null;

    let event = null;
    const ctx = { analysis, pose, recording, pairs, positions, model, distanceM, speedOfSound, referenceWorld, bestWorld, jumpDeg, changedPairs, powers };

    if (dropout && dropoutKey !== this._lastDropoutKey) {
      event = this._raise({
        kind: 'anomaly', cause: 'mic-dropout', stage: 'tdoa',
        reason: `all pairs touching M${dropout.mic} lost coherence and the channel power collapsed; the remaining pairs stay healthy`,
        implicatedMics: [dropout.mic],
        implicatedPairs: dropout.pairs,
      }, ctx);
    } else if (geometryChanged && (solverJumped || statusDrop)) {
      event = this._raise({
        kind: 'expected', cause: 'axis-transform', stage: 'solve',
        reason: 'array geometry or channel layout changed; the direction change is explained by the transform, not by a fault',
      }, ctx);
    } else if (statusDrop && this._allCollapsed(pairs, powers, cfg)) {
      event = this._raise({
        kind: 'expected', cause: 'source-gone', stage: 'tdoa',
        reason: 'every pair coherence collapsed together; the source disappeared rather than a single channel failing',
      }, ctx);
    } else if (note?.sourceChanged && (solverJumped || statusDrop)) {
      event = this._raise({
        kind: 'expected', cause: 'source-moved', stage: 'solve',
        reason: 'the source was moved by the operator; the TDOAs moved consistently with it',
      }, ctx);
      if (bestWorld) this._referenceWorld = bestWorld;
    } else if (solverJumped || statusDrop) {
      event = this._diagnoseSolverJump(ctx);
    } else if (trustedUnsupported && referenceWorld && bestWorld && trustedWorld
      && angularSeparationDeg(trustedWorld, referenceWorld) > cfg.jumpDeg
      && angularSeparationDeg(bestWorld, referenceWorld) <= cfg.jumpDeg) {
      event = this._raise({
        kind: 'anomaly', cause: 'track-switch', stage: 'track',
        reason: 'the solver stayed on the pair-supported reference but the decision bearing left it without pair support',
        reportedWorld: trustedWorld,
      }, ctx);
    }

    this._lastDropoutKey = dropoutKey;
    this._lastStatus = dir.status;
    this._lastPairs = pairs.map((p) => ({ pair: p.pair, lagSec: p.lagSec, sigmaTauSec: p.sigmaTauSec }));

    // --- compact per-frame summary (bounded)
    const supRefFrac = referenceWorld && pairs.length
      ? branchSupport({
        pairs, positions, referenceIndex: m.referenceIndex, speedOfSound,
        directionWorld: referenceWorld, pose, model, distanceM,
        tauSigma: cfg.tauSigma, tauFloorSec: cfg.tauFloorSec,
      }).consistentFrac
      : null;
    this._summaries.push({
      frame: this._frame,
      status: dir.status,
      bestAzEl: dir.best ? { azDeg: dir.best.azDeg, elDeg: dir.best.elDeg } : null,
      jumpDeg: jumpDeg != null ? Number(jumpDeg.toFixed(2)) : null,
      refConsistentFrac: supRefFrac != null ? Number(supRefFrac.toFixed(3)) : null,
      changedPairs: changedPairs.length,
      geometryChanged,
      peaks: peakBrief,
      eventId: event?.id ?? null,
    });
    if (this._summaries.length > cfg.frameSummaryKeep) this._summaries.shift();

    return event;
  }

  _allCollapsed(pairs, powers, cfg) {
    if (!pairs.length) return true;
    const coherenceDead = pairs.every(
      (p) => (p.coherence ?? 1) < cfg.dropoutCoherence || (p.peakToFloor ?? 2) < cfg.dropoutPeakToFloor,
    );
    if (!coherenceDead) return false;
    if (!powers) return true;
    const sorted = [...powers].sort((a, b) => a - b);
    const median = sorted[Math.floor(sorted.length / 2)] || 0;
    return median < 1e-8;
  }

  _diagnoseSolverJump(ctx) {
    const cfg = this.config;
    const { pairs, positions, model, distanceM, speedOfSound, referenceWorld, bestWorld, pose, changedPairs } = ctx;
    const m = ctx.analysis.metadata;
    const supRef = referenceWorld && pairs.length ? branchSupport({
      pairs, positions, referenceIndex: m.referenceIndex, speedOfSound,
      directionWorld: referenceWorld, pose, model, distanceM,
      tauSigma: cfg.tauSigma, tauFloorSec: cfg.tauFloorSec,
    }) : null;
    const supNew = bestWorld && pairs.length ? branchSupport({
      pairs, positions, referenceIndex: m.referenceIndex, speedOfSound,
      directionWorld: bestWorld, pose, model, distanceM,
      tauSigma: cfg.tauSigma, tauFloorSec: cfg.tauFloorSec,
    }) : null;
    ctx.supRef = supRef;
    ctx.supNew = supNew;

    if (referenceWorld && positions.length <= cfg.swapMaxMics) {
      const swap = scanSwap(pairs, positions, m.referenceIndex, referenceWorld, pose, model, distanceM, speedOfSound, cfg);
      if (swap) {
        const [a, b] = swap.mics;
        return this._raise({
          kind: 'anomaly', cause: 'channel-swap', stage: 'tdoa',
          reason: `measured TDOAs fit the reference direction only after exchanging M${a} and M${b} (${Math.round(swap.swappedFrac * 100)}% of pairs vs ${Math.round(swap.identityFrac * 100)}% as mapped); the erroneous delays concentrate on pairs touching M${a}/M${b}`,
          implicatedMics: [a, b],
          implicatedPairs: pairs.filter((p) => p.pair.includes(a) || p.pair.includes(b)).map((p) => p.pair),
          evidence: { swappedFrac: swap.swappedFrac, identityFrac: swap.identityFrac },
        }, ctx);
      }
    }

    if (referenceWorld) {
      const flipped = scanSignError(pairs, positions, m.referenceIndex, referenceWorld, pose, model, distanceM, speedOfSound, m.sampleRate, cfg);
      if (flipped && (!changedPairs.length || flipped.length >= Math.ceil(changedPairs.length / 2))) {
        return this._raise({
          kind: 'anomaly', cause: 'sign-error', stage: 'tdoa',
          reason: `pair(s) ${flipped.map((p) => `M${p[0]}–M${p[1]}`).join(', ')} report tau ~= -tau_predicted and the value is not a peak of the pair's own GCC curve`,
          implicatedPairs: flipped,
          implicatedMics: [...new Set(flipped.flat())],
        }, ctx);
      }
    }

    if (referenceWorld) {
      const refl = scanReflection({
        pairs, positions, referenceIndex: m.referenceIndex, referenceWorld, pose,
        model, distanceM, speedOfSound, sampleRate: m.sampleRate, cfg,
      });
      if (refl) {
        return this._raise({
          kind: 'anomaly', cause: 'strong-reflection', stage: 'tdoa',
          reason: `${refl.badPairs.length} pair(s) keep a direct-path GCC peak but a stronger secondary peak won; the jumped TDOAs fit one coherent direction az ${refl.secondAzEl.azDeg.toFixed(0)}° el ${refl.secondAzEl.elDeg.toFixed(0)}° (wall echo or second coherent source)`,
          implicatedPairs: refl.badPairs,
          implicatedMics: [...new Set(refl.badPairs.flat())],
          evidence: { directAlivePairs: refl.directAlivePairs, secondAzEl: refl.secondAzEl, chi2Red: refl.chi2Red },
        }, ctx);
      }
    }

    const refOk = supRef && supRef.consistentFrac >= cfg.minConsistentFrac;
    const newOk = supNew && supNew.consistentFrac >= cfg.minConsistentFrac;
    if (refOk && newOk) {
      return this._raise({
        kind: 'anomaly', cause: 'multi-peak-ambiguity', stage: 'solve',
        reason: 'the TDOAs did not move and still support the reference branch; the solver ranking flipped between pair-supported candidates',
      }, ctx);
    }
    if (refOk) {
      const stillListed = ctx.analysis.direction.candidates?.some((c) => {
        const w = arrayToWorld(azElToUnit(c.azDeg, c.elDeg), pose.panDeg ?? 0, pose.tiltDeg ?? 0);
        return angularSeparationDeg(w, referenceWorld) <= cfg.jumpDeg;
      });
      return this._raise({
        kind: 'anomaly', cause: 'multi-peak-ambiguity', stage: 'solve',
        reason: stillListed
          ? 'the pairs still support the reference branch and it remains in the candidate set, but the solver demoted it'
          : 'the pairs still support the reference branch but the solver dropped it from the candidate set',
      }, ctx);
    }
    if (newOk) {
      const ev = this._raise({
        kind: 'expected', cause: 'source-moved', stage: 'solve',
        reason: 'the TDOAs moved consistently to the reported direction; a real source motion, not a fault',
      }, ctx);
      if (bestWorld) this._referenceWorld = bestWorld;
      return ev;
    }
    return this._raise({
      kind: 'anomaly', cause: 'unexplained', stage: 'solve',
      reason: 'the direction jumped but no pair-level fault pattern matched; the raw evidence is retained for inspection',
    }, ctx);
  }

  _raise(fields, ctx) {
    const cfg = this.config;
    const micKey = (fields.implicatedMics ?? []).join(',');
    const last = this._events[this._events.length - 1];
    if (last && last.cause === fields.cause && last.kind === fields.kind
      && (last.implicatedMics ?? []).join(',') === micKey
      && this._frame - last.frame <= cfg.mergeFrameGap) {
      last.count = (last.count ?? 1) + 1;
      last.lastFrame = this._frame;
      if (fields.jumpDeg != null) last.jumpDeg = Math.max(last.jumpDeg ?? 0, fields.jumpDeg);
      return null; // merged into the existing event
    }

    const { analysis, pairs, recording, pose } = ctx;
    const m = analysis.metadata;
    const supRef = ctx.supRef ?? (ctx.referenceWorld && pairs.length ? branchSupport({
      pairs, positions: m.positions, referenceIndex: m.referenceIndex, speedOfSound: m.speedOfSound,
      directionWorld: ctx.referenceWorld, pose, model: ctx.model, distanceM: ctx.distanceM,
      tauSigma: cfg.tauSigma, tauFloorSec: cfg.tauFloorSec,
    }) : null);
    const supNew = ctx.supNew ?? (ctx.bestWorld && pairs.length ? branchSupport({
      pairs, positions: m.positions, referenceIndex: m.referenceIndex, speedOfSound: m.speedOfSound,
      directionWorld: ctx.bestWorld, pose, model: ctx.model, distanceM: ctx.distanceM,
      tauSigma: cfg.tauSigma, tauFloorSec: cfg.tauFloorSec,
    }) : null);

    const implicated = fields.implicatedPairs ?? [];
    const peaks = {};
    for (const pr of implicated.slice(0, 6)) {
      const p = pairs.find((x) => x.pair[0] === pr[0] && x.pair[1] === pr[1]);
      if (!p?.lags) continue;
      peaks[pairKeyOf(pr)] = extractPeaks(p, { sampleRate: m.sampleRate, maxPeaks: cfg.peakCount, minProminence: 1.2 })
        .map((pk) => ({ lagSec: pk.lagSec, prominence: Number(pk.prominence.toFixed(2)) }));
    }

    const ev = {
      id: this._nextEventId++,
      frame: this._frame,
      time: new Date().toISOString(),
      kind: fields.kind,
      cause: fields.cause,
      stage: fields.stage,
      reason: fields.reason,
      jumpDeg: ctx.jumpDeg != null ? Number(ctx.jumpDeg.toFixed(2)) : null,
      statusAfter: analysis.direction.status,
      referenceWorld: ctx.referenceWorld ?? null,
      reportedWorld: fields.reportedWorld ?? ctx.bestWorld ?? null,
      referenceAzEl: ctx.referenceWorld ? unitToAzEl(ctx.referenceWorld) : null,
      reportedAzEl: (fields.reportedWorld ?? ctx.bestWorld) ? unitToAzEl(fields.reportedWorld ?? ctx.bestWorld) : null,
      pose: { panDeg: pose.panDeg ?? 0, tiltDeg: pose.tiltDeg ?? 0 },
      implicatedMics: fields.implicatedMics ?? [],
      implicatedPairs: implicated,
      changedPairs: ctx.changedPairs ?? [],
      tauConsistency: supRef ? {
        consistentFrac: Number(supRef.consistentFrac.toFixed(3)),
        perPair: supRef.perPair.map((x) => ({
          pair: x.pair,
          tauMeasSec: x.tauMeasSec,
          tauPredSec: x.tauPredSec,
          z: Number(x.z.toFixed(2)),
          consistent: x.consistent,
        })),
      } : null,
      branchSupport: {
        reference: supRef ? { frac: Number(supRef.consistentFrac.toFixed(3)), count: supRef.count, total: supRef.total } : null,
        reported: supNew ? { frac: Number(supNew.consistentFrac.toFixed(3)), count: supNew.count, total: supNew.total } : null,
        retainable: !!(supRef && supRef.count >= 2 && supRef.consistentFrac >= cfg.minConsistentFrac),
      },
      peaks,
      evidence: fields.evidence ?? null,
      observations: pairs.map((p) => ({
        pair: p.pair,
        tau: p.lagSec,
        sigma: p.sigmaTauSec,
        coherence: p.coherence,
        peakToFloor: p.peakToFloor,
      })),
      solveContext: {
        positions: m.positions.map((p) => p.slice()),
        channelMap: m.channelMap.slice(),
        referenceIndex: m.referenceIndex,
        speedOfSound: m.speedOfSound,
        sampleRate: m.sampleRate,
        startSample: m.startSample,
        frameLength: m.frameLength,
        bandLow: m.bandLow,
        bandHigh: m.bandHigh,
        model: m.model ?? 'auto',
        quality: m.quality,
      },
      segment: null,
      segmentEvicted: false,
      verifyResult: null,
    };

    // Raw multichannel segment retention (mic order), anomalies only, capped.
    if (recording && fields.kind === 'anomaly') {
      ev.segment = {
        channels: m.channelMap.map((recCh) => {
          const ch = recording.channels[recCh];
          return ch ? ch.slice(m.startSample, m.startSample + m.frameLength) : new Float32Array(0);
        }),
        sampleRate: m.sampleRate,
        startSample: m.startSample,
        frameLength: m.frameLength,
      };
      this._segmentOrder.push(ev.id);
      while (this._segmentOrder.length > cfg.maxSegments) {
        const evictId = this._segmentOrder.shift();
        const evict = this._events.find((e) => e.id === evictId);
        if (evict && evict.segment) {
          evict.segment = null;
          evict.segmentEvicted = true;
          this._droppedSegments++;
        }
      }
    }

    this._events.push(ev);
    while (this._events.length > cfg.maxEvents) {
      const old = this._events.shift();
      if (old.segment) {
        this._segmentOrder = this._segmentOrder.filter((id) => id !== old.id);
        this._droppedSegments++;
      }
      this._droppedEvents++;
    }
    return ev;
  }

  events() {
    return [...this._events].reverse();
  }

  latest() {
    return this._events[this._events.length - 1] ?? null;
  }

  summaries() {
    return [...this._summaries];
  }

  stats() {
    return {
      frames: this._frame,
      events: this._events.length,
      anomalies: this._events.filter((e) => e.kind === 'anomaly').length,
      expected: this._events.filter((e) => e.kind === 'expected').length,
      segmentsHeld: this._events.filter((e) => e.segment).length,
      droppedEvents: this._droppedEvents,
      droppedSegments: this._droppedSegments,
      maxEvents: this.config.maxEvents,
      maxSegments: this.config.maxSegments,
    };
  }

  reset() {
    this._events = [];
    this._segmentOrder = [];
    this._summaries = [];
    this._frame = 0;
    this._geoKey = null;
    this._lastPairs = null;
    this._lastStatus = null;
    this._referenceWorld = null;
    this._lastDropoutKey = null;
    this._droppedEvents = 0;
    this._droppedSegments = 0;
  }

  /**
   * Re-run the recorded evidence to verify a diagnosis conclusion:
   *   1. re-solve the stored solver observations (does the fault reproduce?);
   *   2. re-solve with the cause-specific correction (does the reference
   *      direction come back?);
   *   3. when the raw segment was retained, re-run the full pipeline on the
   *      original samples, with and without the correction.
   */
  verify(eventId) {
    const ev = this._events.find((e) => e.id === eventId);
    if (!ev) return null;
    if (ev.verifyResult) return ev.verifyResult;
    const cfg = this.config;
    const ctx = ev.solveContext;
    const pose = ev.pose;
    const toWorld = (best) => (best ? arrayToWorld(azElToUnit(best.azDeg, best.elDeg), pose.panDeg, pose.tiltDeg) : null);
    const errToRef = (world) => (world && ev.referenceWorld ? angularSeparationDeg(world, ev.referenceWorld) : null);
    const errToReported = (world) => (world && ev.reportedWorld ? angularSeparationDeg(world, ev.reportedWorld) : null);
    const hasRefCandidate = (res) => !!res?.candidates?.some((c) => {
      if (c.plausible === false) return false;
      const err = errToRef(toWorld(c));
      return err != null && err <= cfg.verifyTolDeg;
    });
    const out = { eventId, cause: ev.cause, conclusion: 'not-applicable', steps: [] };

    const solve = (observations) => solveDirection({
      observations,
      positions: ctx.positions,
      referenceIndex: ctx.referenceIndex,
      speedOfSound: ctx.speedOfSound,
      model: ctx.model ?? 'auto',
      quality: ctx.quality ?? {},
    });

    // 1. reproduce from the stored observations
    let repro = null;
    try {
      repro = solve(ev.observations.map((o) => ({ ...o })));
      const err = errToReported(toWorld(repro.best));
      out.steps.push({
        step: 'reproduce-observations',
        status: repro.status,
        errorToReportedDeg: err != null ? Number(err.toFixed(2)) : null,
        ok: err != null ? err <= cfg.verifyTolDeg : repro.status === ev.statusAfter,
      });
    } catch (e) {
      out.steps.push({ step: 'reproduce-observations', ok: false, error: e.message });
    }

    // 2. cause-specific correction at the observation level
    let corrected = null;
    let label = null;
    if (ev.cause === 'channel-swap' && ev.implicatedMics.length === 2) {
      const [a, b] = ev.implicatedMics;
      const perm = (mic) => (mic === a ? b : mic === b ? a : mic);
      corrected = ev.observations.map((o) => {
        let [i, j] = [perm(o.pair[0]), perm(o.pair[1])];
        let tau = o.tau;
        if (i > j) { [i, j] = [j, i]; tau = -tau; }
        return { ...o, pair: [i, j], tau };
      });
      label = `exchange the channel-map entries of M${a} and M${b}`;
    } else if (ev.cause === 'mic-dropout' && ev.implicatedMics.length === 1) {
      const mic = ev.implicatedMics[0];
      corrected = ev.observations.filter((o) => o.pair[0] !== mic && o.pair[1] !== mic);
      label = `exclude the pairs touching M${mic}`;
    } else if (ev.cause === 'sign-error') {
      const flip = new Set(ev.implicatedPairs.map(pairKeyOf));
      corrected = ev.observations.map((o) => (flip.has(pairKeyOf(o.pair)) ? { ...o, tau: -o.tau } : o));
      label = 'negate the flagged pair TDOAs';
    } else if (ev.cause === 'strong-reflection' && ev.referenceWorld) {
      const dArr = worldToArray(ev.referenceWorld, pose.panDeg, pose.tiltDeg);
      const offs = predictedOffsets({
        positions: ctx.positions, referenceIndex: ctx.referenceIndex,
        direction: dArr, model: ctx.model === 'near' ? 'near' : 'far',
        distanceM: 1.5, speedOfSound: ctx.speedOfSound,
      });
      corrected = ev.observations.map((o) => {
        const pred = offs[o.pair[0]] - offs[o.pair[1]];
        const tol = Math.max(cfg.tauSigma * (o.sigma ?? 1e-6), cfg.tauFloorSec);
        if (Math.abs(o.tau - pred) <= tol) return o;
        // The direct peak in the echo mixture is pulled off the true direct
        // lag by the echo's cross-spectrum, so re-find it in a wider window
        // (still far below the direct/echo lag separation).
        const matchTol = Math.max(tol, cfg.directPeakMatchSec);
        const cand = (ev.peaks[pairKeyOf(o.pair)] ?? []).find((pk) => Math.abs(pk.lagSec - pred) <= matchTol);
        return cand ? { ...o, tau: cand.lagSec } : o;
      });
      label = 'pick the direct-path GCC peak instead of the reflection';
    }
    if (corrected) {
      try {
        const fixed = solve(corrected);
        const err = errToRef(toWorld(fixed.best));
        const ok = (err != null && err <= cfg.verifyTolDeg) || hasRefCandidate(fixed);
        out.steps.push({
          step: 'corrected-resolve', label, status: fixed.status,
          errorToReferenceDeg: err != null ? Number(err.toFixed(2)) : null, ok,
        });
        out.conclusion = ok ? 'confirmed' : 'refuted';
      } catch (e) {
        out.steps.push({ step: 'corrected-resolve', label, ok: false, error: e.message });
        out.conclusion = 'refuted';
      }
    } else if (ev.cause === 'multi-peak-ambiguity') {
      const ok = !!repro && hasRefCandidate(repro);
      out.steps.push({ step: 'branch-retained', ok, note: 'the reference branch is still among the solver candidates' });
      out.conclusion = ok ? 'confirmed' : 'refuted';
    }

    // 3. end-to-end re-run on the retained raw segment
    if (ev.segment) {
      const runPipeline = (positions, channels) => analyzeDirection({
        recording: { channels, sampleRate: ev.segment.sampleRate, startSample: 0 },
        geometry: {
          positions,
          channelMap: positions.map((_, i) => i),
          speedOfSound: ctx.speedOfSound,
          referenceIndex: Math.min(ctx.referenceIndex, positions.length - 1),
        },
        options: {
          frameLength: ev.segment.frameLength,
          model: ctx.model ?? 'auto',
          bandLow: ctx.bandLow,
          bandHigh: ctx.bandHigh,
        },
      });
      try {
        const rerun = runPipeline(ctx.positions, ev.segment.channels);
        const err = errToReported(toWorld(rerun.direction.best));
        out.steps.push({
          step: 'segment-rerun', status: rerun.direction.status,
          errorToReportedDeg: err != null ? Number(err.toFixed(2)) : null,
          ok: err != null ? err <= cfg.verifyTolDeg : rerun.direction.status === ev.statusAfter,
        });
        if (ev.cause === 'channel-swap' && ev.implicatedMics.length === 2) {
          const [a, b] = ev.implicatedMics;
          const channels = ev.segment.channels.slice();
          [channels[a], channels[b]] = [channels[b], channels[a]];
          const fixedRun = runPipeline(ctx.positions, channels);
          const errF = errToRef(toWorld(fixedRun.direction.best));
          const ok = (errF != null && errF <= cfg.verifyTolDeg) || hasRefCandidate(fixedRun.direction);
          out.steps.push({
            step: 'segment-rerun-corrected', status: fixedRun.direction.status,
            errorToReferenceDeg: errF != null ? Number(errF.toFixed(2)) : null, ok,
          });
          if (out.conclusion !== 'refuted') out.conclusion = ok ? 'confirmed' : out.conclusion;
        }
        if (ev.cause === 'mic-dropout' && ev.implicatedMics.length === 1 && ctx.positions.length > 3) {
          const mic = ev.implicatedMics[0];
          const keep = ctx.positions.map((_, i) => i).filter((i) => i !== mic);
          const fixedRun = runPipeline(
            keep.map((i) => ctx.positions[i]),
            keep.map((i) => ev.segment.channels[i]),
          );
          const ok = hasRefCandidate(fixedRun.direction)
            || (errToRef(toWorld(fixedRun.direction.best)) ?? 999) <= cfg.verifyTolDeg;
          out.steps.push({ step: 'segment-rerun-corrected', status: fixedRun.direction.status, ok });
          if (out.conclusion !== 'refuted') out.conclusion = ok ? 'confirmed' : out.conclusion;
        }
      } catch (e) {
        out.steps.push({ step: 'segment-rerun', ok: false, error: e.message });
      }
    } else {
      out.steps.push({ step: 'segment-rerun', ok: false, note: 'raw segment evicted by the log budget' });
    }

    ev.verifyResult = out;
    return out;
  }
}
