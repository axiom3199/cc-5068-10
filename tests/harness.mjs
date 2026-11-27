import { estimateTdoa } from '../src/core/tdoa.js';
import { azElToUnit, predictedOffsets, unitToAzEl, angularSeparationDeg, azDiffDeg } from '../src/core/geometry.js';
import { renderScene } from '../src/core/synthesize.js';

export const FS = 48000;
export const C = 343;

export function arrays() {
  return {
    // 4-mic square, 12 cm spacing, in the horizontal plane
    square: [
      [-0.06, 0, -0.06],
      [0.06, 0, -0.06],
      [0.06, 0, 0.06],
      [-0.06, 0, 0.06],
    ],
    // 3-mic horizontal triangle, 14 cm
    triangle: [
      [0, 0, 0.08],
      [-0.07, 0, -0.04],
      [0.07, 0, -0.04],
    ],
    // 4-mic linear endfire array (collinear -> cone ambiguity)
    line: [
      [-0.09, 0, 0],
      [-0.03, 0, 0],
      [0.03, 0, 0],
      [0.09, 0, 0],
    ],
    // small tetrahedron, 6 cm, 3D capable
    tetra: [
      [0.0346, 0.0346, 0.0346],
      [0.0346, -0.0346, -0.0346],
      [-0.0346, 0.0346, -0.0346],
      [-0.0346, -0.0346, 0.0346],
    ],
    // tiny 3-cm triangle, worst case for angle resolution
    tiny: [
      [0, 0, 0.03],
      [-0.026, 0, -0.015],
      [0.026, 0, -0.015],
    ],
  };
}

export function tdoaPair(channels, i, j, opts = {}) {
  return estimateTdoa({
    a: channels[i],
    b: channels[j],
    sampleRate: FS,
    frameLength: opts.frameLength ?? 48000,
    maxLagSec: opts.maxLagSec ?? 0.0006,
    bandLow: opts.bandLow ?? 300,
    bandHigh: opts.bandHigh ?? 6000,
    weighting: opts.weighting ?? 'phat',
  });
}

export function sceneFor(positions, az, el, extra = {}) {
  const src = { azDeg: az, elDeg: el, distanceM: extra.distanceM ?? null, nearField: extra.nearField ?? false };
  return renderScene({
    positions,
    source: src,
    sampleRate: FS,
    speedOfSound: C,
    durationSec: extra.durationSec ?? 1.5,
    snrDb: extra.snrDb ?? 25,
    seed: extra.seed ?? 7,
    bandLow: extra.bandLow ?? 300,
    bandHigh: extra.bandHigh ?? 6000,
    equalPower: true,
  });
}

export function truePairTaus(positions, az, el, model, distanceM) {
  const taus = predictedOffsets({
    positions,
    referenceIndex: 0,
    direction: { azDeg: az, elDeg: el },
    model,
    distanceM,
    speedOfSound: C,
  });
  const out = [];
  for (let i = 0; i < positions.length; i++) {
    for (let j = i + 1; j < positions.length; j++) out.push({ i, j, tau: taus[i] - taus[j] });
  }
  return out;
}

export function check(cond, msg) {
  if (!cond) throw new Error(msg);
}

export function approx(a, b, tol, msg) {
  if (!(Math.abs(a - b) <= tol)) throw new Error(`${msg}: |${a} - ${b}| > ${tol}`);
}

export function reportDirection(res) {
  if (!res.best) return '<no estimate>';
  return `az=${res.best.azDeg.toFixed(1)} el=${res.best.elDeg.toFixed(1)} ${res.model} conf=${res.confidence.toFixed(2)} ${res.status}${res.reason ? ` (${res.reason})` : ''}`;
}

export function angleError(res, az, el) {
  const truth = azElToUnit(az, el);
  const est = azElToUnit(res.best.azDeg, res.best.elDeg);
  return angularSeparationDeg(est, truth);
}
