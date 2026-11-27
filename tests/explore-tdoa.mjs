import { arrays, sceneFor, tdoaPair, truePairTaus, FS, C } from './harness.mjs';

const A = arrays();
const positions = A.square;
for (const [az, el] of [[0, 0], [30, 0], [90, 0], [150, 10], [250, 25]]) {
  const sc = sceneFor(positions, az, el, { snrDb: 25 });
  const truth = truePairTaus(positions, az, el, 'far');
  console.log(`\naz=${az} el=${el}`);
  for (const { i, j, tau } of truth) {
    const r = tdoaPair(sc.channels, i, j);
    const errUs = (r.lagSec - tau) * 1e6;
    console.log(` pair ${i}${j}: true=${(tau * 1e6).toFixed(2)}us est=${(r.lagSec * 1e6).toFixed(2)}us err=${errUs.toFixed(2)}us coh=${r.coherence.toFixed(3)} ptf=${r.peakToFloor.toFixed(1)}`);
  }
}
