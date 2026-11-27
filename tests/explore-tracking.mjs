import { runTrackingTrial } from './tracking-eval.mjs';

const planar = [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]];
const stereo = [[-0.07, 0.004, 0], [0.07, -0.004, 0]];
const tetra = [[0.05, 0.04, 0.03], [0.05, -0.03, -0.04], [-0.05, 0.03, -0.04], [-0.05, -0.04, 0.03]];

function show(name, r) {
  const m = r.metrics;
  console.log(`${name}: acq=${m.acquisitionSec?.toFixed(2)}s lock=${(m.lockRatio * 100).toFixed(0)}% wrongSwitches=${m.wrongSwitchCount} jumps=${m.aimJumpCount} medErr=${m.medianReportedErrorDeg?.toFixed(1)}° p90=${m.p90ReportedErrorDeg?.toFixed(1)}°`);
}

console.log('--- 1. static source, 3D array (should slew and lock)');
show('tetra', runTrackingTrial({
  positions: tetra,
  segments: [{ tStart: 0, tEnd: 4, azDeg: 55, elDeg: 10 }],
  durationSec: 4,
}));

console.log('--- 2. planar array, source elevated, must report ambiguity and hold until rotation');
show('planar', runTrackingTrial({
  positions: planar,
  segments: [{ tStart: 0, tEnd: 4, azDeg: 50, elDeg: 25 }],
  durationSec: 4,
}));

console.log('--- 3. two-mic array (cone ambiguity everywhere)');
show('stereo', runTrackingTrial({
  positions: stereo,
  segments: [{ tStart: 0, tEnd: 4, azDeg: 45, elDeg: 0 }],
  durationSec: 4,
}));

console.log('--- 4. source starts, then disappears (must hold, not invent)');
show('tetra-gone', runTrackingTrial({
  positions: tetra,
  segments: [
    { tStart: 0, tEnd: 2, azDeg: 40, elDeg: 5 },
    { tStart: 2, tEnd: 4, azDeg: 40, elDeg: 5, present: false },
  ],
  durationSec: 4,
}));

console.log('--- 5. moving source');
show('tetra-move', runTrackingTrial({
  positions: tetra,
  segments: [{ tStart: 0, tEnd: 4, azDeg: 20, elDeg: 0, dAz: 60, dEl: 10 }],
  durationSec: 4,
}));
