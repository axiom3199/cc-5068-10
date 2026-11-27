// Single entry point for turning a synchronized multichannel recording into a
// direction estimate. Everything the inference path may use is explicit here:
//   - channel order:       channelMap[i] = recording channel used as mic i
//   - sample timing:       sampleRate, startSample, per-channel calibration delays
//   - propagation speed:   speedOfSound (m/s)
//   - array geometry:      mic positions in metres, array-frame reference channel
// No ground-truth field is accepted, read, or forwarded. The input object is
// copied, so later mutation of the caller's scene cannot leak into the result.

import { estimateAllPairs, observationsFromPairs } from './tdoa.js';
import { solveDirection } from './solver.js';
import { fft, ifft, nextPow2 } from './fft.js';

export const DEFAULT_QUALITY = Object.freeze({
  minCoherence: 0.12,
  minPeakToFloor: 1.8,
  minEvidence: 0.22,
  maxReducedChi2: 12,
  maxPairs: 64,
  ambiguityDeltaChi2: 9.21,
  systematicTdoaSec: 5e-7,
});

export function validateInput(o) {
  const errs = [];
  if (!o || typeof o !== 'object') throw new Error('analysis input must be an object');
  const { recording, geometry, options = {} } = o;
  if (!recording) throw new Error('recording is required');
  const chs = recording.channels;
  if (!Array.isArray(chs) || chs.length === 0) throw new Error('recording.channels must be a non-empty array');
  for (let i = 0; i < chs.length; i++) {
    if (!chs[i] || typeof chs[i].length !== 'number') throw new Error(`recording.channels[${i}] is not array-like`);
  }
  if (!(recording.sampleRate > 0)) throw new Error('recording.sampleRate must be a positive number (Hz)');
  if (!Number.isFinite(options.bandHigh ?? 0) && options.bandHigh != null) throw new Error('bandHigh must be finite');
  if (options.bandHigh != null && options.bandLow != null && options.bandHigh <= options.bandLow) {
    throw new Error('bandHigh must exceed bandLow');
  }
  if (!geometry) throw new Error('geometry is required');
  const pos = geometry.positions;
  if (!Array.isArray(pos) || pos.length < 2) throw new Error('geometry.positions needs at least 2 microphones');
  for (let m = 0; m < pos.length; m++) {
    if (!Array.isArray(pos[m]) || pos[m].length !== 3 || pos[m].some((v) => !Number.isFinite(v))) {
      throw new Error(`geometry.positions[${m}] must be [x,y,z] in metres`);
    }
  }
  const chMap = geometry.channelMap ?? pos.map((_, i) => i);
  if (!Array.isArray(chMap) || chMap.length !== pos.length) {
    throw new Error('geometry.channelMap must map every microphone to one recording channel');
  }
  const used = new Set();
  for (const c of chMap) {
    if (!Number.isInteger(c) || c < 0 || c >= chs.length) throw new Error(`channelMap index ${c} out of range`);
    if (used.has(c)) throw new Error(`channel ${c} is mapped to more than one microphone`);
    used.add(c);
  }
  const c = geometry.speedOfSound;
  if (!(c > 100 && c < 2000)) throw new Error(`speedOfSound must be within 100..2000 m/s (got ${c})`);
  const sr = recording.sampleRate;
  if (sr < 4000 || sr > 768000) errs.push(`unusual sample rate ${sr} Hz`);
  if (geometry.channelDelaysSec) {
    if (geometry.channelDelaysSec.length !== pos.length) throw new Error('channelDelaysSec must have one entry per microphone');
    for (const d of geometry.channelDelaysSec) if (!Number.isFinite(d)) throw new Error('channelDelaysSec entries must be finite');
  }
  for (let i = 0; i < pos.length; i++) {
    for (let j = i + 1; j < pos.length; j++) {
      const d = Math.hypot(pos[i][0] - pos[j][0], pos[i][1] - pos[j][1], pos[i][2] - pos[j][2]);
      if (d < 1e-6) throw new Error(`microphones ${i} and ${j} are coincident`);
    }
  }
  return errs;
}

/** Shift a channel by a fractional number of samples using an FFT phase ramp. */
function shiftChannel(ch, delaySamples) {
  if (!delaySamples) return ch;
  const n = ch.length;
  const nfft = nextPow2(n);
  const re = new Float64Array(nfft);
  const im = new Float64Array(nfft);
  re.set(ch);
  fft(re, im);
  for (let k = 0; k < nfft / 2; k++) {
    const w = (2 * Math.PI * k * delaySamples) / nfft;
    const cw = Math.cos(w);
    const sw = Math.sin(w);
    const rr = re[k] * cw + im[k] * sw;
    const ii = im[k] * cw - re[k] * sw;
    re[k] = rr;
    im[k] = ii;
    if (k > 0) {
      re[nfft - k] = rr;
      im[nfft - k] = -ii;
    }
  }
  ifft(re, im);
  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) out[i] = re[i];
  return out;
}

/**
 * @param {object} o
 * @param {object} o.recording   { channels, sampleRate, startSample? }
 * @param {object} o.geometry    { positions, channelMap?, channelDelaysSec?, referenceIndex?, speedOfSound }
 * @param {object} [o.options]   analysis band, window, model, quality thresholds
 * @returns {object} direction result with explicit metadata and per-pair TDOAs
 */
export function analyzeDirection(o) {
  const warnings = validateInput(o);
  const { recording, geometry, options = {} } = o;
  const sampleRate = recording.sampleRate;
  const positions = geometry.positions.map((p) => p.slice());
  const channelMap = (geometry.channelMap ?? positions.map((_, i) => i)).slice();
  const channelDelaysSec = (geometry.channelDelaysSec ?? positions.map(() => 0)).slice();
  const referenceIndex = geometry.referenceIndex ?? 0;
  if (referenceIndex < 0 || referenceIndex >= positions.length) throw new Error('referenceIndex out of range');
  const speedOfSound = geometry.speedOfSound;
  const quality = { ...DEFAULT_QUALITY, ...(options.quality ?? {}) };
  const startSample = Math.max(0, Math.floor(recording.startSample ?? options.startSample ?? 0));
  const available = Math.min(...channelMap.map((c) => recording.channels[c].length)) - startSample;
  if (available < 64) throw new Error(`analysis window has only ${available} samples`);
  const frameLength = Math.max(128, Math.min(
    Math.floor(options.frameLength ?? Math.min(available, Math.round(0.12 * sampleRate))),
    Math.floor(available),
  ));

  const channels = channelMap.map((c, m) => {
    const src = recording.channels[c];
    const seg = new Float32Array(frameLength);
    seg.set(src.subarray(startSample, startSample + frameLength));
    // Calibration delay: a positive value means this channel's samples are late,
    // so advance it to align the time base across channels.
    return shiftChannel(seg, -channelDelaysSec[m] * sampleRate);
  });

  const maxBaseline = (() => {
    let m = 0;
    for (let i = 0; i < positions.length; i++) {
      for (let j = i + 1; j < positions.length; j++) {
        m = Math.max(m, Math.hypot(
          positions[i][0] - positions[j][0],
          positions[i][1] - positions[j][1],
          positions[i][2] - positions[j][2],
        ));
      }
    }
    return m;
  })();
  const maxLagSec = options.maxLagSec ?? Math.min(0.02, (maxBaseline / speedOfSound) * 1.15 + 2 / sampleRate);

  const pairEst = estimateAllPairs({
    channels,
    positions,
    sampleRate,
    speedOfSound,
    frameLength,
    bandLow: options.bandLow ?? Math.max(120, Math.round(sampleRate * 0.006)),
    bandHigh: options.bandHigh ?? Math.min(7000, Math.round(sampleRate * 0.45)),
    weighting: options.weighting ?? 'phat',
    maxPairs: quality.maxPairs,
    maxLagSec,
    preWindow: options.preWindow,
  });

  const pairs = pairEst.pairs.map((p) => {
    // Fold the systematic time-base uncertainty into each pair variance.
    const sig = Math.hypot(p.sigmaTauSec, quality.systematicTdoaSec);
    return { ...p, sigmaTauSec: sig };
  });
  const observations = observationsFromPairs(pairs);

  const direction = solveDirection({
    observations,
    positions,
    referenceIndex,
    speedOfSound,
    model: options.model ?? 'auto',
    distanceM: options.nearDistanceM ?? 1.5,
    grid: options.grid,
    quality,
    arrayGeometry: options.arrayGeometry,
  });

  // The solver has no idea how many pairs a two-microphone rig can produce; make
  // the cone limitation explicit here so a two-mic result never looks like a fit
  // that merely lacked data.
  if (positions.length === 2 && direction.status === 'insufficient') {
    direction.candidates = [];
    direction.reason = 'two microphones constrain only one inter-channel TDOA, which leaves a cone of possible directions; no unique bearing can be reported';
    direction.ambiguity = { isAmbiguous: true, type: 'cone', separationDeg: null, cone: null };
    direction.geometry = direction.geometry ?? { kind: 'collinear', rank: 1 };
  }

  return {
    direction,
    tdoa: {
      pairs: pairs.map((p) => ({
        pair: p.pair,
        lagSec: p.lagSec,
        lagSamples: p.lagSamples,
        sigmaTauSec: p.sigmaTauSec,
        coherence: p.coherence,
        peakToFloor: p.peakToFloor,
        peakValue: p.peakValue,
        lags: p.lags,
        values: p.values,
        maxLag: p.maxLag,
        bandLow: p.bandLow,
        bandHigh: p.bandHigh,
        weighting: p.weighting,
      })),
      maxLagSec,
    },
    metadata: {
      sampleRate,
      startSample,
      frameLength,
      frameDurationSec: frameLength / sampleRate,
      speedOfSound,
      channelMap,
      channelDelaysSec,
      referenceIndex,
      positions,
      bandLow: options.bandLow ?? Math.max(120, Math.round(sampleRate * 0.006)),
      bandHigh: options.bandHigh ?? Math.min(7000, Math.round(sampleRate * 0.45)),
      weighting: options.weighting ?? 'phat',
      model: direction.model,
      quality,
      warnings,
    },
  };
}

/**
 * Baseline that the task explicitly warns against: treating the loudest channel as
 * the direction. Provided for comparison/validation only; never used for estimates.
 * It points from the array centroid toward the loudest mic.
 */
export function loudestChannelBearing(recording, geometry, frameLength = 0.1) {
  const n = Math.floor(recording.sampleRate * frameLength);
  const map = geometry.channelMap ?? geometry.positions.map((_, i) => i);
  let best = -1;
  let bestPow = -Infinity;
  map.forEach((c, m) => {
    const ch = recording.channels[c];
    let p = 0;
    for (let i = 0; i < Math.min(n, ch.length); i++) p += ch[i] * ch[i];
    if (p > bestPow) { bestPow = p; best = m; }
  });
  const p = geometry.positions[best];
  const nrm = Math.hypot(p[0], p[1], p[2]);
  if (nrm < 1e-9) return { mic: best, azDeg: 0, elDeg: 0 };
  const az = Math.atan2(p[0], p[2]) * 180 / Math.PI;
  const el = Math.asin(p[1] / nrm) * 180 / Math.PI;
  return { mic: best, azDeg: (az + 360) % 360, elDeg: el };
}
