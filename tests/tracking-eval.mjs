// End-to-end tracking evaluation with a synthetic locked object.
//
// The inference path receives ONLY mic channels, sample timing, array geometry,
// channel map, speed of sound and the camera pose. Ground truth (source azimuth /
// elevation / distance) is used only by the scorer, never by analyzeDirection().
//
// Metrics (all computed from the locked-object ground truth):
//   - acquisition time: seconds from source onset to first time the true source is
//     inside the FOV and the camera has settled;
//   - lock ratio: fraction of "source audible" time the true source is inside the FOV;
//   - false switcher: number of times the camera target jumps to a wrong direction
//     (a candidate > wrongSwitchDeg from truth held for >= wrongSwitchFrames frames);
//   - peak angular error: reported best direction vs truth, when reported;
//   - mirror resolution: for a planar array, frames taken to exclude the false
//     mirror after the rig rotates.

import { renderScene } from '../src/core/synthesize.js';
import { analyzeDirection } from '../src/core/analysis.js';
import { DirectionTracker } from '../src/core/tracker.js';
import { VirtualCamera, CameraController } from '../src/core/camera.js';
import { azElToUnit, angularSeparationDeg, unitToAzEl } from '../src/core/geometry.js';

const FS = 48000;
const C = 343;

function sceneAt(positions, worldAz, worldEl, panDeg, tiltDeg, opts) {
  // The source is fixed in the world; the mics rotate with the camera, so in the
  // array frame the source direction is R^T * d_world.
  const d = azElToUnit(worldAz, worldEl);
  const y = panDeg * Math.PI / 180;
  const t = tiltDeg * Math.PI / 180;
  const cy = Math.cos(y), sy = Math.sin(y), ct = Math.cos(t), st = Math.sin(t);
  const R = [
    [cy, -sy * st, sy * ct],
    [0, ct, st],
    [-sy, -cy * st, cy * ct],
  ];
  const local = [
    R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
    R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
    R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
  ];
  const ae = unitToAzEl(local);
  return renderScene({
    positions,
    source: { azDeg: ae.azDeg, elDeg: ae.elDeg },
    sampleRate: FS,
    speedOfSound: C,
    durationSec: opts.chunkSec ?? 0.5,
    snrDb: opts.snrDb ?? 22,
    seed: opts.seed ?? 1,
    bandLow: 300,
    bandHigh: 6000,
    equalPower: true,
  });
}

/**
 * Run a trajectory: a list of world segments. Each segment is
 *   { tStart, tEnd, azDeg, elDeg, snrDb?, present?:boolean }
 * The camera runs at 1/60 s, a new inference frame every analysisEverySec.
 */
export function runTrackingTrial(cfg) {
  const {
    positions,
    segments,
    durationSec,
    cameraConfig = {},
    analysisEverySec = 0.25,
    chunkSec = 0.5,
    seed = 3,
    arrayGeometryExtra = {},
  } = cfg;

  const camera = new VirtualCamera(cameraConfig, cfg.initialPose ?? {});
  const tracker = new DirectionTracker(cfg.trackerConfig ?? {});
  const controller = new CameraController(camera);
  const dt = 1 / 60;
  const frames = Math.round(durationSec / dt);
  const analysisFrames = Math.round(analysisEverySec / dt);

  const truthAt = (t) => {
    for (const s of segments) {
      if (t >= s.tStart && t < s.tEnd) {
        const u = (t - s.tStart) / Math.max(1e-6, s.tEnd - s.tStart);
        return {
          azDeg: s.azDeg + (s.dAz ?? 0) * u,
          elDeg: s.elDeg + (s.dEl ?? 0) * u,
          snrDb: s.snrDb ?? cfg.snrDb ?? 22,
          present: s.present !== false,
        };
      }
    }
    return null;
  };

  const events = [];
  let lastAnalysis = -Infinity;
  let lastResult = null;
  let lastDecision = null;
  let inFovFrames = 0;
  let audibleFrames = 0;
  let firstLockT = null;
  let settledAtLock = null;
  let wrongSwitchCount = 0;
  let wrongRun = 0;
  let prevAim = null;
  let aimJumpTimes = [];
  let lastAimWorld = null;
  const lockSamples = [];

  for (let f = 0; f < frames; f++) {
    const t = f * dt;
    const truth = truthAt(t);
    const pan = camera.yawDeg;
    const tilt = camera.tiltDeg;

    if (t - lastAnalysis >= analysisEverySec - 1e-9) {
      lastAnalysis = t;
      if (truth && truth.present) {
        const sc = sceneAt(positions, truth.azDeg, truth.elDeg, pan, tilt, {
          chunkSec, snrDb: truth.snrDb, seed: seed + Math.round(t * 10),
        });
        const analysis = analyzeDirection({
          recording: { channels: sc.channels, sampleRate: FS },
          geometry: {
            positions,
            speedOfSound: C,
            referenceIndex: 0,
            ...arrayGeometryExtra,
          },
          options: {
            frameLength: Math.round(0.12 * FS),
            model: 'auto',
            bandLow: 300,
            bandHigh: 6000,
          },
        });
        lastResult = analysis.direction;
        lastDecision = tracker.update(lastResult, { panDeg: pan, tiltDeg: tilt }, t);
      } else {
        lastResult = {
          status: 'insufficient',
          reason: 'source not present',
          best: null,
          candidates: [],
          confidence: 0,
        };
        lastDecision = tracker.update(lastResult, { panDeg: pan, tiltDeg: tilt }, t);
      }
      controller.command(lastDecision);
      events.push({
        t,
        status: lastResult.status,
        reason: lastResult.reason,
        action: lastDecision.action,
        pan,
        tilt,
        truth: truth ? { azDeg: truth.azDeg, elDeg: truth.elDeg } : null,
        best: lastResult.best ? { azDeg: lastResult.best.azDeg, elDeg: lastResult.best.elDeg } : null,
        candidates: lastResult.candidates?.map((c) => ({ azDeg: c.azDeg, elDeg: c.elDeg, deltaChi2: c.deltaChi2 })) ?? [],
      });
    }

    const stepRes = controller.step(dt);

    if (truth && truth.present) {
      audibleFrames++;
      const truthDir = azElToUnit(truth.azDeg, truth.elDeg);
      const seen = camera.sees(truthDir);
      if (seen) {
        inFovFrames++;
        if (!stepRes.slewing) {
          if (firstLockT == null) {
            firstLockT = t;
            settledAtLock = t;
          }
          settledAtLock = t;
        }
      }
      lockSamples.push(seen ? 1 : 0);

      // Wrong-switch detection: does the commanded aim sit far from truth?
      const aimWorld = lastDecision?.bearing?.world ?? null;
      if (aimWorld) {
        const aimAng = unitToAzEl(aimWorld);
        const err = angularSeparationDeg(azElToUnit(aimAng.azDeg, aimAng.elDeg), truthDir);
        if (err > (cfg.wrongSwitchDeg ?? 35)) {
          wrongRun++;
          if (wrongRun === (cfg.wrongSwitchFrames ?? 3)) wrongSwitchCount++;
        } else {
          wrongRun = 0;
        }
        if (lastAimWorld) {
          const jump = angularSeparationDeg(aimWorld, lastAimWorld);
          if (jump > (cfg.jumpDeg ?? 30)) aimJumpTimes.push(t);
        }
        lastAimWorld = aimWorld;
      }
    }
    prevAim = camera.yawDeg;
  }

  // Angular error of reported best bearings (array frame -> world comparison done
  // at the time of the report using the pose stored in each event).
  const reportedErrors = [];
  for (const e of events) {
    if (!e.best || !e.truth) continue;
    // Recover world angle: e.best is in the array frame; rotate by the pose recorded.
    const y = e.pan * Math.PI / 180;
    const tt = e.tilt * Math.PI / 180;
    const cy = Math.cos(y), sy = Math.sin(y), ct = Math.cos(tt), st = Math.sin(tt);
    const R = [
      [cy, -sy * st, sy * ct],
      [0, ct, st],
      [-sy, -cy * st, cy * ct],
    ];
    const dl = azElToUnit(e.best.azDeg, e.best.elDeg);
    const dw = [
      R[0][0] * dl[0] + R[0][1] * dl[1] + R[0][2] * dl[2],
      R[1][0] * dl[0] + R[1][1] * dl[1] + R[1][2] * dl[2],
      R[2][0] * dl[0] + R[2][1] * dl[1] + R[2][2] * dl[2],
    ];
    reportedErrors.push(angularSeparationDeg(dw, azElToUnit(e.truth.azDeg, e.truth.elDeg)));
  }
  reportedErrors.sort((a, b) => a - b);

  return {
    events,
    metrics: {
      acquisitionSec: firstLockT,
      lockRatio: audibleFrames > 0 ? inFovFrames / audibleFrames : 0,
      audibleSec: audibleFrames * dt,
      inFovSec: inFovFrames * dt,
      wrongSwitchCount,
      aimJumpCount: aimJumpTimes.length,
      medianReportedErrorDeg: reportedErrors.length
        ? reportedErrors[Math.floor(reportedErrors.length / 2)]
        : null,
      p90ReportedErrorDeg: reportedErrors.length
        ? reportedErrors[Math.min(reportedErrors.length - 1, Math.floor(reportedErrors.length * 0.9))]
        : null,
      finalPose: { pan: camera.yawDeg, tilt: camera.tiltDeg },
      finalTrusted: tracker.getConfirmedWorld(),
      tracks: tracker.tracks.map((t) => ({
        azDeg: unitToAzEl(t.direction).azDeg,
        elDeg: unitToAzEl(t.direction).elDeg,
        weight: t.weight,
        hits: t.hits,
      })),
    },
  };
}
