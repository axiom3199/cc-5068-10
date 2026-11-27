// Monte-Carlo calibration of reported TDOA sigma vs actual error.
import { arrays, sceneFor, truePairTaus, FS, C } from './harness.mjs';
import { estimateAllPairs, observationsFromPairs } from '../src/core/tdoa.js';

const A = arrays();
for (const [name, snr] of [['square', 30], ['square', 15], ['square', 5], ['tiny', 20], ['tiny', 5]]) {
  const positions = A[name];
  const zs = [];
  const errs = [];
  for (let seed = 1; seed <= 24; seed++) {
    const sc = sceneFor(positions, 37, 8, { snrDb: snr, seed, durationSec: 1.0 });
    const truth = truePairTaus(positions, 37, 8, 'far');
    const est = estimateAllPairs({
      channels: sc.channels, positions, sampleRate: FS, speedOfSound: C,
      frameLength: 48000, bandLow: 300, bandHigh: 6000, maxLagSec: 0.0006,
    });
    const obs = observationsFromPairs(est.pairs);
    for (const o of obs) {
      const t = truth.find((q) => q.i === o.pair[0] && q.j === o.pair[1]);
      errs.push((o.tau - t.tau) * 1e6);
      zs.push((o.tau - t.tau) / o.sigma);
    }
  }
  const rmsErr = Math.sqrt(errs.reduce((s, e) => s + e * e, 0) / errs.length);
  const rmsZ = Math.sqrt(zs.reduce((s, e) => s + e * e, 0) / zs.length);
  const meanSigma = Math.sqrt(zs.length) / 0; // placeholder
  console.log(`${name}@${snr}dB: tdoa rms err=${rmsErr.toFixed(2)}us  rms(z)=${rmsZ.toFixed(2)} (want ~1)`);
}
