import { renderScene } from '../src/core/synthesize.js';
import { analyzeDirection } from '../src/core/analysis.js';
import { azElToUnit, arrayToWorld, unitToAzEl, angularSeparationDeg } from '../src/core/geometry.js';

const FS = 48000, C = 343;
// vertical planar array: mics in the x-y plane (normal +z) -> front/back ambiguity
const verticalPlanar = [[-0.06, -0.06, 0], [0.06, -0.06, 0], [0.06, 0.06, 0], [-0.06, 0.06, 0]];
const horizontalPlanar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];

function local(worldAz, worldEl, pan, tilt) {
  const y = pan * Math.PI / 180, t = tilt * Math.PI / 180;
  const cy = Math.cos(y), sy = Math.sin(y), ct = Math.cos(t), st = Math.sin(t);
  const R = [[cy, -sy * st, sy * ct], [0, ct, st], [-sy, -cy * st, cy * ct]];
  const d = azElToUnit(worldAz, worldEl);
  return unitToAzEl([R[0][0]*d[0]+R[1][0]*d[1]+R[2][0]*d[2], R[0][1]*d[0]+R[1][1]*d[1]+R[2][1]*d[2], R[0][2]*d[0]+R[1][2]*d[1]+R[2][2]*d[2]]);
}

function candidatesWorld(pos, worldAz, worldEl, pan, tilt) {
  const la = local(worldAz, worldEl, pan, tilt);
  const sc = renderScene({ positions: pos, source: { azDeg: la.azDeg, elDeg: la.elDeg }, sampleRate: FS, speedOfSound: C, durationSec: 0.5, snrDb: 25, seed: 7, bandLow: 300, bandHigh: 6000, equalPower: true });
  const a = analyzeDirection({ recording: { channels: sc.channels, sampleRate: FS }, geometry: { positions: pos, speedOfSound: C }, options: { frameLength: 5760, model: 'far', bandLow: 300, bandHigh: 6000 } });
  return { status: a.direction.status, cands: a.direction.candidates.map(c => {
    const w = arrayToWorld(azElToUnit(c.azDeg, c.elDeg), pan, tilt);
    const ae = unitToAzEl(w);
    return { world: w, az: ae.azDeg, el: ae.elDeg, dchi: c.deltaChi2 };
  })};
}

for (const [name, pos, az, el] of [
  ['vertical planar, pan sweep', verticalPlanar, 35, 10],
  ['horizontal planar, pan sweep', horizontalPlanar, 35, 25],
  ['horizontal planar, tilt sweep', horizontalPlanar, 35, 25],
]) {
  console.log(`\n=== ${name}  world(${az},${el})`);
  const truth = azElToUnit(az, el);
  const sweep = name.includes('tilt') ? [0, 3, 6, 9, 12] : [0, 6, 12, 18, 24];
  for (const p of sweep) {
    const pan = name.includes('tilt') ? 10 : p;
    const tilt = name.includes('tilt') ? p : 0;
    const r = candidatesWorld(pos, az, el, pan, tilt);
    console.log(` pan=${String(pan).padStart(2)} tilt=${String(tilt).padStart(2)} ${r.status.padEnd(10)} ` +
      r.cands.map(c => `world(${c.az.toFixed(0)},${c.el.toFixed(0)}) err=${angularSeparationDeg(c.world, truth).toFixed(0)} d=${c.dchi.toFixed(1)}`).join(' | '));
  }
}
