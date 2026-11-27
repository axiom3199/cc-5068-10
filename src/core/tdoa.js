// GCC-PHAT time-difference-of-arrival estimation for multichannel recordings.
//
// Convention: estimateTdoa(a, b) returns tau such that a(t) ~= b(t - tau), i.e. the
// signal in channel a arrives LATER than in channel b by tau seconds. This matches
// geometry.predictedOffsets(): tau_ij = (arrival offset of i) - (arrival offset of j).
//
// Peak refinement uses parabolic interpolation. Each pair carries quality metrics
// (coherence, peak-to-floor, peak width) used later to decide whether the observation
// supports a direction at all.

import { fft, ifft, nextPow2 } from './fft.js';

export function hannWindow(n) {
  const w = new Float64Array(n);
  for (let i = 0; i < n; i++) w[i] = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (n - 1)));
  return w;
}

/** Precompute windowed spectra for every channel so cross-spectra are cheap. */
export function channelSpectra(o) {
  const { channels, startSample = 0, sampleRate } = o;
  const n = o.frameLength ?? Math.min(...channels.map((c) => c.length - startSample));
  if (n < 64) throw new Error('segment too short for TDOA');
  const nfft = nextPow2(Math.max(o.nfftMin ?? 4096, n));
  const win = o.preWindow === false ? null : hannWindow(n);
  const spectra = channels.map((ch) => {
    let mean = 0;
    for (let i = 0; i < n; i++) mean += ch[startSample + i];
    mean /= n;
    const re = new Float64Array(nfft);
    const im = new Float64Array(nfft);
    for (let i = 0; i < n; i++) re[i] = (ch[startSample + i] - mean) * (win ? win[i] : 1);
    fft(re, im);
    return { re, im };
  });
  return { spectra, nfft, frameLength: n, startSample, sampleRate };
}

function bandBinsFor(nfft, sampleRate, bandLow, bandHigh) {
  return { kLow: Math.max(1, Math.ceil((bandLow * nfft) / sampleRate)), kHigh: Math.min(nfft / 2 - 1, Math.floor((bandHigh * nfft) / sampleRate)) };
}

/**
 * Cross-correlation between two channels of a spectra bundle.
 * @returns {{lagSec:number, lagSamples:number, peakValue:number, peakToFloor:number,
 *            coherence:number, peakHalfWidthSamples:number, lags:Int32Array,
 *            values:Float64Array, maxLag:number, ...}}
 */
export function pairTdoa(bundle, i, j, o = {}) {
  const { spectra, nfft, sampleRate } = bundle;
  const bandLow = o.bandLow ?? 150;
  const bandHigh = o.bandHigh ?? Math.min(7000, sampleRate * 0.47);
  const weighting = o.weighting === 'gcc' ? 'gcc' : 'phat';
  const maxLagSec = o.maxLagSec ?? 0.005;
  const maxLag = Math.min(Math.floor(nfft / 2) - 2, Math.ceil(maxLagSec * sampleRate));
  const { kLow, kHigh } = bandBinsFor(nfft, sampleRate, bandLow, bandHigh);
  if (kHigh <= kLow) throw new Error('empty analysis band');

  const A = spectra[i];
  const B = spectra[j];
  const cr = new Float64Array(nfft / 2);
  const ci = new Float64Array(nfft / 2);
  let crossMag = 0;
  let powA = 0;
  let powB = 0;
  for (let k = kLow; k <= kHigh; k++) {
    const ar = A.re[k], ai = A.im[k];
    const br = B.re[k], bi = B.im[k];
    const xr = ar * br + ai * bi; // A * conj(B)
    const xi = ai * br - ar * bi;
    cr[k] = xr;
    ci[k] = xi;
    crossMag += Math.hypot(xr, xi);
    powA += ar * ar + ai * ai;
    powB += br * br + bi * bi;
  }
  // Magnitude-squared coherence, bounded [0,1] by Cauchy-Schwarz.
  const coherence = powA > 0 && powB > 0 ? Math.min(1, (crossMag * crossMag) / (powA * powB)) : 0;
  const bins = kHigh - kLow + 1;
  const reg = 0.01 * (crossMag / bins) + 1e-12;

  const reC = new Float64Array(nfft);
  const imC = new Float64Array(nfft);
  for (let k = kLow; k <= kHigh; k++) {
    const xr = cr[k];
    const xi = ci[k];
    const wgt = weighting === 'phat' ? 1 / (Math.hypot(xr, xi) + reg) : 1;
    reC[k] = xr * wgt;
    imC[k] = xi * wgt;
    reC[nfft - k] = reC[k];
    imC[nfft - k] = -imC[k];
  }
  ifft(reC, imC);

  const lagCount = maxLag * 2 + 1;
  const lags = new Int32Array(lagCount);
  const values = new Float64Array(lagCount);
  let peakIdx = 0;
  let peakVal = -Infinity;
  for (let t = 0; t < lagCount; t++) {
    const lag = t - maxLag;
    lags[t] = lag;
    const v = reC[lag < 0 ? lag + nfft : lag];
    values[t] = v;
    if (v > peakVal) { peakVal = v; peakIdx = t; }
  }

  let delta = 0;
  if (peakIdx > 0 && peakIdx < lagCount - 1) {
    const ym = values[peakIdx - 1];
    const y0 = values[peakIdx];
    const yp = values[peakIdx + 1];
    const denom = ym - 2 * y0 + yp;
    if (Math.abs(denom) > 1e-15) delta = (0.5 * (ym - yp)) / denom;
    if (!Number.isFinite(delta) || Math.abs(delta) > 0.5) delta = 0;
  }
  const lagSamples = lags[peakIdx] + delta;
  const lagSec = lagSamples / sampleRate;

  // Floor from samples outside the main lobe; peak-to-floor is the sharpness score.
  let sq = 0;
  let cnt = 0;
  for (let t = 0; t < lagCount; t++) {
    if (Math.abs(t - peakIdx) <= 3) continue;
    sq += values[t] * values[t];
    cnt++;
  }
  const floorRms = cnt > 0 ? Math.sqrt(sq / cnt) : 0;
  const peakToFloor = peakVal / (floorRms + 1e-12);
  const halfLevel = peakVal / Math.max(2, peakToFloor);
  let halfWidth = 0;
  for (let t = peakIdx + 1; t < lagCount; t++) {
    if (values[t] <= halfLevel) { halfWidth = t - peakIdx; break; }
  }
  if (halfWidth === 0) halfWidth = lagCount - peakIdx;
  // Half-width to sigma for a Gaussian main lobe.
  const widthSigmaSamples = halfWidth / 1.1774;

  // Analytic TDOA standard deviation for a band-limited, incoherent-noise case:
  //   sigma_tau ~= 1 / (2*pi*f_rms*sqrt(T*B*SNR_eff))
  // f_rms is the power-weighted RMS frequency in the analysis band, T the frame
  // duration, B the bandwidth, and SNR_eff is approximated by the lag-domain
  // peak-to-floor ratio. The constant is calibrated (and deliberately kept
  // slightly conservative) against Monte-Carlo runs in tests/calibration.
  let f2sum = 0;
  for (let k = kLow; k <= kHigh; k++) {
    const f = (k * sampleRate) / nfft;
    f2sum += f * f;
  }
  const fRms = Math.sqrt(f2sum / Math.max(1, kHigh - kLow + 1));
  const bandwidth = Math.max(1, bandHigh - bandLow);
  const frameSec = bundle.frameLength / sampleRate;
  // Two independent sigma estimates, take the larger:
  //  - analytic CRLB-like term from SNR and bandwidth;
  //  - empirical term calibrated by Monte-Carlo against this estimator:
  //      sigma_us ~= 6.3 * (1 - coherence)^0.73  for the 0.3-6 kHz band.
  //    Coherence is the better low-SNR predictor because it folds in the
  //    in-band noise that PHAT weights equally across bins.
  const snrLike = Math.max(1.05, peakToFloor);
  const analyticSigma = 1 / (2 * Math.PI * fRms * Math.sqrt(Math.max(1e-12, frameSec * bandwidth * snrLike)));
  const empiricalSigma = 6.3e-6 * Math.pow(Math.max(1e-6, 1 - coherence), 0.733);
  const finiteLobe = (widthSigmaSamples / sampleRate) / Math.max(1e-6, (crossMag / Math.max(1, bins)) * frameSec);
  const sigmaTauSec = Math.max(analyticSigma, empiricalSigma, 0.06e-6);
  const sigmaTauSecConservative = Math.min(2e-4, sigmaTauSec * (o.sigmaSafety ?? 1.4));

  return {
    pair: [i, j],
    lagSec,
    lagSamples,
    peakValue: peakVal,
    peakToFloor,
    coherence,
    peakHalfWidthSamples: halfWidth,
    widthSigmaSamples,
    sigmaTauSec: sigmaTauSecConservative,
    fRms,
    snrLike,
    bandLow,
    bandHigh,
    weighting,
    lags,
    values,
    maxLag,
    nfft,
    frameLength: bundle.frameLength,
    startSample: bundle.startSample,
    sampleRate,
    scoreDb: peakToFloor > 1 ? 20 * Math.log10(peakToFloor) : 0,
  };
}

/** Single-pair convenience wrapper (test tooling, UI pair inspector). */
export function estimateTdoa(o) {
  const bundle = channelSpectra({
    channels: [o.a, o.b],
    sampleRate: o.sampleRate,
    frameLength: o.frameLength,
    nfftMin: o.nfftMin,
    preWindow: o.preWindow,
  });
  const r = pairTdoa(bundle, 0, 1, o);
  r.pair = [0, 1];
  return r;
}

/**
 * Estimate TDOAs for a chosen list of pairs.
 * @param {object} o
 * @param {Float32Array[]} o.channels
 * @param {number[][]} o.positions
 * @param {number} o.sampleRate
 * @param {number} o.speedOfSound
 * @param {number} [o.startSample=0]
 * @param {number} [o.frameLength]
 * @param {number[][]} [o.pairs] defaults to all i<j
 * @param {number} [o.maxPairs=32]
 */
export function estimateAllPairs(o) {
  const { channels, positions, sampleRate, speedOfSound } = o;
  const bundle = channelSpectra({
    channels,
    sampleRate,
    startSample: o.startSample ?? 0,
    frameLength: o.frameLength,
    nfftMin: o.nfftMin,
    preWindow: o.preWindow,
  });
  let pairs = o.pairs;
  if (!pairs) {
    pairs = [];
    for (let i = 0; i < channels.length; i++) for (let j = i + 1; j < channels.length; j++) pairs.push([i, j]);
  } else {
    pairs = pairs.filter(([i, j]) => i >= 0 && j >= 0 && i < channels.length && j < channels.length && i !== j);
  }
  // Rank pairs by baseline length, then cap.
  const base = (p) => {
    const a = positions[p[0]], b = positions[p[1]];
    return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  };
  pairs = [...pairs].sort((p, q) => base(q) - base(p)).slice(0, o.maxPairs ?? 32);

  const maxLagSec = o.maxLagSec ?? (o.frameLength ?? bundle.frameLength) / sampleRate;
  const results = pairs.map(([i, j]) => pairTdoa(bundle, i, j, {
    bandLow: o.bandLow,
    bandHigh: o.bandHigh,
    weighting: o.weighting,
    maxLagSec,
  }));
  return { pairs: results, bundle, sampleRate, speedOfSound };
}

/** Convert pair estimates into the solver's observation list with per-pair sigma. */
export function observationsFromPairs(pairResults, opts = {}) {
  const sigmaFloor = opts.sigmaFloorSec ?? 1e-7;
  const sigmaMax = opts.sigmaMaxSec ?? 2e-4;
  return pairResults.map((r) => ({
    pair: r.pair,
    tau: r.lagSec,
    sigma: Math.max(sigmaFloor, Math.min(sigmaMax, r.sigmaTauSec ?? sigmaFloor)),
    coherence: r.coherence,
    peakToFloor: r.peakToFloor,
    weighting: r.weighting,
  }));
}
