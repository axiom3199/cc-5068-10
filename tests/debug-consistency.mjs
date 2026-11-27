import { runScriptedDisambiguation } from './probe-eval.mjs';

const planar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
const r = runScriptedDisambiguation({
  positions: planar,
  truth: { azDeg: 50, elDeg: 25 },
  poses: [0, 6, 12, 18, 24].map((tiltDeg) => ({ panDeg: 0, tiltDeg })),
});
for (const s of r.steps) {
  console.log(`tilt ${s.pose.tiltDeg} ${s.status} action=${s.decision.action} reason="${s.decision.reason}" consistent=${s.consistent ? `(${s.consistent.azDeg.toFixed(1)},${s.consistent.elDeg.toFixed(1)})` : 'null'}`);
}
console.log('tracks', r.tracker.tracks.map((t) => ({ az: t.azDeg.toFixed(1), el: t.elDeg.toFixed(1), w: t.weight.toFixed(2), hits: t.hits, conf: t.confirmed })));
