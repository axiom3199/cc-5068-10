import { renderScene } from '../src/core/synthesize.js';
import { estimateAllPairs, observationsFromPairs } from '../src/core/tdoa.js';
import { solveDirection } from '../src/core/solver.js';
import { azElToUnit, predictedOffsets } from '../src/core/geometry.js';
import { mulberry32 } from '../src/core/synthesize.js';

const FS = 48000, C = 343;
const arrays = {
  bar: [[-0.09,0.004,0],[-0.03,-0.004,0],[0.03,0.004,0],[0.09,-0.004,0]],
  barNoWobble: [[-0.09,0,0],[-0.03,0,0],[0.03,0,0],[0.09,0,0]],
  zigzagZ: [[0,0.004,-0.09],[0,-0.004,-0.03],[0,0.004,0.03],[0,-0.004,0.09]],
  shallowTetra: [[0.06,0.01,0.04],[-0.05,0.01,-0.05],[-0.01,0.02,0.07],[0.02,0.05,-0.02]],
  cameraTop: [[-0.06,0.02,-0.03],[0.06,0.02,-0.03],[0.0,0.02,0.07],[0.0,0.035,0.0]],
};

function anglesFor(positions, truthAz, truthEl, panDeg) {
  // rotate world truth into array frame and rotate mics to world; equivalent to
  // solving in the rotated array frame with local truth direction.
  const pan = panDeg * Math.PI / 180;
  const R = [[Math.cos(pan),0,Math.sin(pan)],[0,1,0],[-Math.sin(pan),0,Math.cos(pan)]];
  const d = azElToUnit(truthAz, truthEl);
  const local = [R[0][0]*d[0]+R[1][0]*d[1]+R[2][0]*d[2], R[0][1]*d[0]+R[1][1]*d[1]+R[2][1]*d[2], R[0][2]*d[0]+R[1][2]*d[1]+R[2][2]*d[2]];
  const az = (Math.atan2(local[0],local[2])*180/Math.PI+360)%360;
  const el = Math.asin(local[1])*180/Math.PI;
  return { az, el };
}

function run(name, positions, truthAz, truthEl, panDeg) {
  const local = anglesFor(positions, truthAz, truthEl, panDeg);
  const sc = renderScene({ positions, source:{azDeg:local.az, elDeg:local.el}, sampleRate:FS, speedOfSound:C, durationSec:1.2, snrDb:25, seed:11, bandLow:300, bandHigh:6000, equalPower:true });
  const est = estimateAllPairs({ channels: sc.channels, positions, sampleRate:FS, speedOfSound:C, frameLength:48000, bandLow:300, bandHigh:6000, maxLagSec:0.0007 });
  const obs = observationsFromPairs(est.pairs);
  const res = solveDirection({ observations:obs, positions, referenceIndex:0, speedOfSound:C, model:'far', allowNear:false });
  const cands = res.candidates.map(c=>`(${c.azDeg.toFixed(0)},${c.elDeg.toFixed(0)}) d${c.deltaChi2.toFixed(1)}`).join(' ');
  console.log(`${name.padEnd(12)} pan${String(panDeg).padStart(3)} world(${truthAz},${truthEl}) local(${local.az.toFixed(1)},${local.el.toFixed(1)}) -> ${res.status} ${res.model} chi2Red=${res.fit?.chi2Red?.toFixed(2)} cond=${res.conditioning?.condition?.toExponential(1)} cands: ${cands}`);
}

for (const [name, pos] of Object.entries(arrays)) {
  console.log('==', name);
  for (const pan of [0, 30, 60]) run(name, pos, 60, 0, pan);
  run(name, pos, 150, 20, 0);
}
