import { runProbeTrial, runScriptedDisambiguation } from './probe-eval.mjs';

const horizontalPlanar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
const tetra = [[0.05, 0.04, 0.03], [0.05, -0.03, -0.04], [-0.05, 0.03, -0.04], [-0.05, -0.04, 0.03]];

console.log('== scripted: horizontal planar at world (50, 25), tilt the rig to separate up/down mirror');
const sc = runScriptedDisambiguation({
  positions: horizontalPlanar,
  truth: { azDeg: 50, elDeg: 25 },
  poses: [
    { panDeg: 0, tiltDeg: 0 },
    { panDeg: 0, tiltDeg: 8 },
    { panDeg: 0, tiltDeg: 16 },
    { panDeg: 0, tiltDeg: 20 },
  ],
});
for (const s of sc.steps) {
  console.log(` pose tilt=${s.pose.tiltDeg} ${s.status} cands=${s.candidates.length} ` +
    `action=${s.decision.action} trustedWorld=${s.decision.bearing ? `(${s.decision.bearing.azDeg.toFixed(0)},${s.decision.bearing.elDeg.toFixed(0)})` : 'none'} ` +
    `consistent=${s.consistent ? `(${s.consistent.azDeg.toFixed(0)},${s.consistent.elDeg.toFixed(0)})` : 'none'}`);
}

console.log('\n== closed loop probe: horizontal planar at world (50, 25)');
const r = runProbeTrial({
  positions: horizontalPlanar,
  truth: { azDeg: 50, elDeg: 25 },
  durationSec: 9,
  planeNormalArray: [0, 1, 0],
  probeConfig: { probeDeg: 15, armAfterSec: 0.7 },
});
console.log(JSON.stringify(r.metrics, null, 2));

console.log('\n== closed loop probe: tetrahedron (no ambiguity expected, should just lock)');
const r2 = runProbeTrial({
  positions: tetra,
  truth: { azDeg: 50, elDeg: 15 },
  durationSec: 6,
  planeNormalArray: [0, 1, 0],
});
console.log(JSON.stringify(r2.metrics, null, 2));
