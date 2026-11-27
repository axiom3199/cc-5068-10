import { arrays, sceneFor, truePairTaus, FS, C, reportDirection, angleError } from './harness.mjs';
import { estimateAllPairs, observationsFromPairs } from '../src/core/tdoa.js';
import { solveDirection } from '../src/core/solver.js';

const A = arrays();
const cases = [
  ['square', [0, 0]],
  ['square', [30, 20]],
  ['square', [120, -15]],
  ['triangle', [45, 0]],
  ['tetra', [200, 35]],
  ['line', [60, 0]],
  ['tiny', [0, 0]],
];

for (const [name, [az, el]] of cases) {
  const positions = A[name];
  const sc = sceneFor(positions, az, el, { snrDb: 25 });
  const est = estimateAllPairs({
    channels: sc.channels,
    positions,
    sampleRate: FS,
    speedOfSound: C,
    frameLength: 48000,
    bandLow: 300,
    bandHigh: 6000,
    maxLagSec: 0.0006,
  });
  const obs = observationsFromPairs(est.pairs);
  const res = solveDirection({
    observations: obs,
    positions,
    referenceIndex: 0,
    speedOfSound: C,
    model: 'far',
    allowNear: true,
  });
  const err = res.best ? angleError(res, az, el) : NaN;
  console.log(`${name.padEnd(8)} truth ${String(az).padStart(3)},${String(el).padStart(3)}  ->  ${reportDirection(res)}  err=${err.toFixed(2)}° cands=${res.candidates.length} cond=${res.conditioning?.condition?.toFixed(1)}`);
  if (res.candidates.length > 1) {
    for (const c of res.candidates.slice(1)) console.log(`    alt az=${c.azDeg.toFixed(1)} el=${c.elDeg.toFixed(1)} dChi2=${c.deltaChi2.toFixed(1)}`);
  }
  if (res.credible) console.log(`    credible az ${res.credible.azRangeDeg.map(v=>v.toFixed(0))} (${res.credible.azExtentDeg.toFixed(0)}°) el ${res.credible.elRangeDeg.map(v=>v.toFixed(0))} (${res.credible.elExtentDeg.toFixed(0)}°)`);
  console.log(`    chi2Red=${res.fit?.chi2Red?.toFixed(2)} sigmaAz=${res.best?.sigmaAzDeg?.toFixed(2)} sigmaEl=${res.best?.sigmaElDeg?.toFixed(2)}`);
}
