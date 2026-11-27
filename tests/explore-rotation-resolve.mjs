// Verify the mechanism by which camera rotation can exclude a mirror branch.
//
// Claim: for a (near-)planar array, the array-frame mirror solution maps to a
// DIFFERENT world direction at each camera pose, while the true source is the same
// world direction every frame. So a world-frame tracker can keep the consistent
// hypothesis and drop the one that jumps around as the camera pans.
//
// This script shows both candidate world directions at several fixed pans, plus the
// per-frame solver status. It does not implement tracking policy; it validates the
// physics the policy will rely on.

import { renderScene } from '../src/core/synthesize.js';
import { analyzeDirection } from '../src/core/analysis.js';
import { azElToUnit, unitToAzEl, arrayToWorld, angularSeparationDeg } from '../src/core/geometry.js';

const FS = 48000, C = 343;
const planar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
const nearPlanar = [[-0.06, 0.006, -0.06], [0.06, -0.006, -0.06], [0.06, 0.006, 0.06], [-0.06, -0.006, 0.06]];

function localAzEl(worldAz, worldEl, pan, tilt) {
  const y = pan * Math.PI / 180, t = tilt * Math.PI / 180;
  const cy = Math.cos(y), sy = Math.sin(y), ct = Math.cos(t), st = Math.sin(t);
  const R = [[cy, -sy * st, sy * ct], [0, ct, st], [-sy, -cy * st, cy * ct]];
  const d = azElToUnit(worldAz, worldEl);
  const l = [R[0][0]*d[0]+R[1][0]*d[1]+R[2][0]*d[2], R[0][1]*d[0]+R[1][1]*d[1]+R[2][1]*d[2], R[0][2]*d[0]+R[1][2]*d[1]+R[2][2]*d[2]];
  return unitToAzEl(l);
}

for (const [name, pos] of [['exact planar', planar], ['near planar', nearPlanar]]) {
  console.log(`\n=== ${name}, world source (70, 25)`);
  const truth = azElToUnit(70, 25);
  for (const pan of [0, 15, 30, 45, 60]) {
    const la = localAzEl(70, 25, pan, 0);
    const sc = renderScene({ positions: pos, source: { azDeg: la.azDeg, elDeg: la.elDeg }, sampleRate: FS, speedOfSound: C, durationSec: 0.6, snrDb: 25, seed: 5, bandLow: 300, bandHigh: 6000, equalPower: true });
    const a = analyzeDirection({ recording: { channels: sc.channels, sampleRate: FS }, geometry: { positions: pos, speedOfSound: C }, options: { frameLength: 5760, model: 'far', bandLow: 300, bandHigh: 6000 } });
    const d = a.direction;
    const worlds = d.candidates.map((c) => {
      const w = arrayToWorld(azElToUnit(c.azDeg, c.elDeg), pan, 0);
      const ae = unitToAzEl(w);
      return { ae, errToTruth: angularSeparationDeg(w, truth), dchi: c.deltaChi2 };
    });
    console.log(` pan=${String(pan).padStart(2)} local(${la.azDeg.toFixed(1)},${la.elDeg.toFixed(1)}) ${d.status.padEnd(10)} ` +
      worlds.map((w) => `world(${w.ae.azDeg.toFixed(0)},${w.ae.elDeg.toFixed(0)}) err=${w.errToTruth.toFixed(0)} dχ²=${w.dchi.toFixed(1)}`).join('  |  '));
  }
}
