// Does rotating the rig (camera pan) change the residual asymmetry between the
// mirror hypotheses for a slightly non-planar array? And for an exactly planar one?
import { renderScene } from '../src/core/synthesize.js';
import { estimateAllPairs, observationsFromPairs } from '../src/core/tdoa.js';
import { solveDirection } from '../src/core/solver.js';
import { azElToUnit, rotationMatrix, rotateVec, unitToAzEl, angularSeparationDeg } from '../src/core/geometry.js';

const FS = 48000, C = 343;

function localDirectionFor(worldAz, worldEl, panDeg, tiltDeg) {
  // d_local = R^T d_world
  const R = rotationMatrix(panDeg, tiltDeg);
  const d = azElToUnit(worldAz, worldEl);
  const local = [
    R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
    R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
    R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
  ];
  return unitToAzEl(local);
}

function toWorld(az, el, panDeg, tiltDeg) {
  const R = rotationMatrix(panDeg, tiltDeg);
  const d = azElToUnit(az, el);
  return rotateVec(R, d);
}

function trial(name, positions, worldAz, worldEl, pans, snrDb = 30, seed = 9) {
  console.log(`\n== ${name}  world source (${worldAz}, ${worldEl})`);
  for (const pan of pans) {
    const local = localDirectionFor(worldAz, worldEl, pan, 0);
    const sc = renderScene({
      positions, source: { azDeg: local.azDeg, elDeg: local.elDeg },
      sampleRate: FS, speedOfSound: C, durationSec: 1.2, snrDb, seed,
      bandLow: 300, bandHigh: 6000, equalPower: true,
    });
    const est = estimateAllPairs({
      channels: sc.channels, positions, sampleRate: FS, speedOfSound: C,
      frameLength: 48000, bandLow: 300, bandHigh: 6000, maxLagSec: 0.0007,
    });
    const obs = observationsFromPairs(est.pairs);
    const res = solveDirection({ observations: obs, positions, referenceIndex: 0, speedOfSound: C, model: 'far' });
    const cands = res.candidates.map((c) => ({
      az: c.azDeg, el: c.elDeg, chi2: c.chi2,
      world: unitToAzEl(toWorld(c.azDeg, c.elDeg, pan, 0)),
    }));
    const best = cands[0];
    const alt = cands[1];
    const truthSepBest = angularSeparationDeg(toWorld(best.az, best.el, pan, 0), azElToUnit(worldAz, worldEl));
    const truthSepAlt = alt ? angularSeparationDeg(toWorld(alt.az, alt.el, pan, 0), azElToUnit(worldAz, worldEl)) : null;
    console.log(` pan=${String(pan).padStart(3)} local(${local.azDeg.toFixed(1)},${local.elDeg.toFixed(1)}) ${res.status} kind=${res.geometry.kind} ` +
      `bestWorld(${best.world.azDeg.toFixed(0)},${best.world.elDeg.toFixed(0)}) errToTruth=${truthSepBest.toFixed(1)} ` +
      (alt ? `altWorld(${alt.world.azDeg.toFixed(0)},${alt.world.elDeg.toFixed(0)}) sep=${angularSeparationDeg(toWorld(best.az, best.el, pan, 0), toWorld(alt.az, alt.el, pan, 0)).toFixed(0)} dChi2=${(alt.chi2 - best.chi2).toFixed(2)} altErrToTruth=${truthSepAlt.toFixed(1)}` : 'no alt'));
  }
}

// exactly planar array (horizontal plane)
const planar = [[-0.05, 0, -0.05], [0.05, 0, -0.05], [0.05, 0, 0.05], [-0.05, 0, 0.05]];
// slightly non-planar: ±6 mm out of plane
const nearPlanar = [[-0.05, 0.006, -0.05], [0.05, -0.006, -0.05], [0.05, 0.006, 0.05], [-0.05, -0.006, 0.05]];
// vertical planar array (in x-y plane, normal +z)
const vertical = [[-0.05, -0.05, 0], [0.05, -0.05, 0], [0.05, 0.05, 0], [-0.05, 0.05, 0]];
// slightly non-planar vertical
const nearVertical = [[-0.05, -0.05, 0.006], [0.05, -0.05, -0.006], [0.05, 0.05, 0.006], [-0.05, 0.05, -0.006]];

trial('exact planar (horizontal)', planar, 70, 30, [0, 40, 80, 120]);
trial('near planar (horizontal)', nearPlanar, 70, 30, [0, 40, 80, 120]);
trial('exact vertical planar', vertical, 70, 20, [0, 40, 80, 120]);
trial('near vertical planar', nearVertical, 70, 20, [0, 40, 80, 120]);
