// Synthetic multichannel audio for validation and the interactive demo.
// This module is the ONLY place that knows source ground truth. Inference
// modules receive channels + geometry only (see tests/inference-isolation).

import { fft, ifft, nextPow2 } from './fft.js';

export function mulberry32(seed) {
  let a = seed >>> 0;
  return function rng() {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function gaussian(rng) {
  let u = 0;
  let v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// Band-limited noise with a gentle 1/f tilt, generated in the frequency domain.
function makeSourceNoise(rng, n, sampleRate, bandLow, bandHigh, tilt = 0.35) {
  const M = nextPow2(n);
  const re = new Float64Array(M);
  const im = new Float64Array(M);
  for (let k = 1; k < M / 2; k++) {
    const f = (k * sampleRate) / M;
    if (f < bandLow || f > bandHigh) continue;
    const mag = Math.pow(f / 1000, -tilt) * (0.6 + 0.8 * rng());
    const ph = 2 * Math.PI * rng();
    re[k] = mag * Math.cos(ph);
    im[k] = mag * Math.sin(ph);
    re[M - k] = re[k];
    im[M - k] = -im[k];
  }
  ifft(re, im);
  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) out[i] = re[i];
  return out;
}

export function azElToUnit(azDeg, elDeg) {
  const az = (azDeg * Math.PI) / 180;
  const el = (elDeg * Math.PI) / 180;
  const ce = Math.cos(el);
  return [Math.sin(az) * ce, Math.sin(el), Math.cos(az) * ce];
}

export function unitToAzEl([x, y, z]) {
  const n = Math.hypot(x, y, z) || 1;
  const u = [x / n, y / n, z / n];
  const el = (Math.asin(Math.max(-1, Math.min(1, u[1]))) * 180) / Math.PI;
  const az = (Math.atan2(u[0], u[2]) * 180) / Math.PI;
  return { azDeg: (az + 360) % 360, elDeg: el };
}

function normalize(v) {
  const n = Math.hypot(...v);
  return n > 0 ? v.map((x) => x / n) : v;
}

/**
 * Render a multichannel recording of one source.
 * @param {object} cfg
 * @param {number[][]} cfg.positions  mic positions in the ARRAY frame (m)
 * @param {object} cfg.source  { azDeg, elDeg, distanceM, nearField?:boolean }
 * @param {number} cfg.sampleRate
 * @param {number} cfg.speedOfSound
 * @param {number} cfg.durationSec
 * @param {number} [cfg.snrDb=20]  per-channel SNR relative to source power
 * @param {number} [cfg.seed=1]
 * @param {number} [cfg.bandLow=250] [cfg.bandHigh=6500]
 * @param {boolean} [cfg.equalPower=false] far-field sensors share power
 * @returns {{channels: Float32Array[], sampleRate:number, truth:object}}
 */
export function renderScene(cfg) {
  const {
    positions,
    source,
    sampleRate,
    speedOfSound,
    durationSec,
    snrDb = 20,
    seed = 1,
    bandLow = 250,
    bandHigh = 6500,
    equalPower = false,
  } = cfg;
  const rng = mulberry32(seed);
  const n = Math.max(512, Math.round(durationSec * sampleRate));
  const c = speedOfSound;
  const d = azElToUnit(source.azDeg, source.elDeg);
  const near = !!source.nearField;
  const distance = Math.max(0.05, source.distanceM ?? 2);

  const delays = positions.map((p) => (near || source.distanceM != null ? Math.hypot(distance * d[0] - p[0], distance * d[1] - p[1], distance * d[2] - p[2]) / c : -(d[0] * p[0] + d[1] * p[1] + d[2] * p[2]) / c));
  const gains = positions.map((p) => {
    if (!near && source.distanceM == null) return 1;
    const r = Math.hypot(distance * d[0] - p[0], distance * d[1] - p[1], distance * d[2] - p[2]);
    return Math.max(0.05, r) / Math.max(0.05, distance);
  });
  const minDelay = Math.min(...delays);
  const relDelays = delays.map((t) => t - minDelay);
  const maxDelay = Math.max(...relDelays);
  const maxDelaySamples = Math.ceil(maxDelay * sampleRate) + 2;
  const preRoll = Math.round(0.02 * sampleRate);
  const signalLen = n;
  const fftLen = nextPow2(signalLen + maxDelaySamples + preRoll + 64);

  const base = makeSourceNoise(rng, signalLen, sampleRate, bandLow, bandHigh);
  const baseRe = new Float64Array(fftLen);
  const baseIm = new Float64Array(fftLen);
  baseRe.set(base);
  fft(baseRe, baseIm);

  const channels = [];
  for (let m = 0; m < positions.length; m++) {
    const re = new Float64Array(fftLen);
    const im = new Float64Array(fftLen);
    const shiftSamples = (relDelays[m] * sampleRate) + preRoll;
    for (let k = 0; k <= fftLen / 2; k++) {
      const w = (2 * Math.PI * k * shiftSamples) / fftLen;
      const cw = Math.cos(w);
      const sw = Math.sin(w);
      // e^{-j w shift}: delay
      re[k] = baseRe[k] * cw + baseIm[k] * sw;
      im[k] = baseIm[k] * cw - baseRe[k] * sw;
      if (k > 0 && k < fftLen / 2) {
        re[fftLen - k] = re[k];
        im[fftLen - k] = -im[k];
      }
    }
    ifft(re, im);
    const ch = new Float32Array(signalLen + maxDelaySamples + preRoll);
    for (let i = 0; i < ch.length; i++) ch[i] = re[i] * gains[m];
    channels.push(ch);
  }

  // Equalize source power per channel and add independent noise.
  let sumPow = 0;
  let count = 0;
  for (const ch of channels) {
    for (let i = 0; i < ch.length; i++) sumPow += ch[i] * ch[i];
    count += ch.length;
  }
  const rms = Math.sqrt(sumPow / Math.max(1, count));
  const target = 0.2;
  const scale = rms > 0 ? target / rms : 0;
  const noiseSigma = scale > 0 ? (target / Math.pow(10, snrDb / 20)) : 0;
  for (let m = 0; m < channels.length; m++) {
    const ch = channels[m];
    for (let i = 0; i < ch.length; i++) {
      ch[i] = ch[i] * scale + noiseSigma * gaussian(rng);
    }
  }

  return {
    channels,
    sampleRate,
    truth: {
      azDeg: source.azDeg,
      elDeg: source.elDeg,
      distanceM: source.distanceM ?? null,
      nearField: near,
      unit: d,
      positions,
      speedOfSound,
      seed,
      snrDb,
      equalPower,
    },
  };
}
