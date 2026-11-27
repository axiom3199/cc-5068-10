// End-to-end tracking + disambiguation evaluation.
//
// The inference path sees ONLY mic channels, timing, geometry, channel map, speed of
// sound and the camera pose. Ground truth is used only by the scorer. Three things
// are measured against the locked object:
//   1. acquisition + lock ratio,
//   2. wrong-switch count,
//   3. whether a deliberate probe rotation excludes the mirror branch (mirror
//      resolution time + whether the final trusted bearing is the true one).

import { renderScene } from '../src/core/synthesize.js';
import { analyzeDirection } from '../src/core/analysis.js';
import { DirectionTracker } from '../src/core/tracker.js';
import { VirtualCamera, CameraController } from '../src/core/camera.js';
import { planProbeForDecision } from '../src/core/disambiguation.js';
import { azElToUnit, angularSeparationDeg, unitToAzEl } from '../src/core/geometry.js';

const FS = 48000;
const C = 343;

function poseRotateInverse(worldAz, worldEl, pan, tilt) {
  const d = azElToUnit(worldAz, worldEl);
  const y = pan * Math.PI / 180, t = tilt * Math.PI / 180;
  const cy = Math.cos(y), sy = Math.sin(y), ct = Math.cos(t), st = Math.sin(t);
  const R = [[cy, -sy * st, sy * ct], [0, ct, st], [-sy, -cy * st, cy * ct]];
  return unitToAzEl([
    R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
    R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
    R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
  ]);
}

export function arrayLocalFor(worldAz, worldEl, pan, tilt) {
  return poseRotateInverse(worldAz, worldEl, pan, tilt);
}

export function renderAt(positions, truth, pan, tilt, opts = {}) {
  const l = poseRotateInverse(truth.azDeg, truth.elDeg, pan, tilt);
  const near = truth.nearField;
  return renderScene({
    positions,
    source: {
      azDeg: l.azDeg,
      elDeg: l.elDeg,
      nearField: !!near,
      distanceM: near ? truth.distanceM : null,
    },
    sampleRate: FS,
    speedOfSound: C,
    durationSec: opts.chunkSec ?? 0.6,
    snrDb: opts.snrDb ?? 22,
    seed: opts.seed ?? 1,
    bandLow: opts.bandLow ?? 300,
    bandHigh: opts.bandHigh ?? 6000,
    equalPower: !near,
  });
}

export function analyzeAt(positions, truth, pan, tilt, opts = {}) {
  const sc = renderAt(positions, truth, pan, tilt, opts);
  const analysis = analyzeDirection({
    recording: { channels: sc.channels, sampleRate: FS },
    geometry: { positions, speedOfSound: C, referenceIndex: 0 },
    options: {
      frameLength: Math.round((opts.frameSec ?? 0.12) * FS),
      model: opts.model ?? 'auto',
      bandLow: opts.bandLow ?? 300,
      bandHigh: opts.bandHigh ?? 6000,
      nearDistanceM: truth.distanceM ?? 1.5,
    },
  });
  return { analysis, scene: sc };
}

/**
 * Run a scripted sequence of "observations" with explicit camera poses, feeding the
 * tracker and returning what it did at each step. Used to verify that a later
 * observation excludes a branch (not merely that a peak angle moved).
 */
export function runScriptedDisambiguation({ positions, truth, poses, probePlan = null, opts = {} }) {
  const tracker = new DirectionTracker(opts.trackerConfig ?? {});
  const steps = [];
  for (let i = 0; i < poses.length; i++) {
    const pose = poses[i];
    const { analysis } = analyzeAt(positions, truth, pose.panDeg, pose.tiltDeg, opts);
    const decision = tracker.update(analysis.direction, { panDeg: pose.panDeg, tiltDeg: pose.tiltDeg }, pose.t ?? i);
    steps.push({
      pose,
      status: analysis.direction.status,
      candidates: analysis.direction.candidates.map((c) => ({ azDeg: c.azDeg, elDeg: c.elDeg, deltaChi2: c.deltaChi2, plausible: c.plausible })),
      decision: { action: decision.action, reason: decision.reason, bearing: decision.bearing },
      consistent: tracker.consistentWorld() ? unitToAzEl(tracker.consistentWorld()) : null,
    });
  }
  return { steps, tracker };
}

/**
 * Full closed-loop trial with a probe: when the tracker is stuck holding an
 * ambiguous family, the driver asks disambiguation.js which reorientation will
 * separate the branches, performs it, and checks whether the family collapses.
 */
export function runProbeTrial(cfg) {
  const {
    positions,
    truth,
    durationSec = 8,
    dt = 1 / 60,
    analysisEverySec = 0.35,
    probeConfig = {},
    trackerConfig = {},
    cameraConfig = {},
    arrayGeometryExtra = {},
    seed = 4,
  } = cfg;

  const planeNormalArray = cfg.planeNormalArray ?? [0, 1, 0];
  const camera = new VirtualCamera(cameraConfig, cfg.initialPose ?? {});
  const tracker = new DirectionTracker(trackerConfig);
  const controller = new CameraController(camera);
  const events = [];
  let probe = null;
  let probeStart = null;
  let probeDone = null;
  let analysisClock = 0;
  let lastAnalysis = -Infinity;
  let lastDecision = null;
  let lastAnalysisResult = null;
  let mirrorResolvedAt = null;
  let finalTrustedError = null;
  let probePredicted = null;
  let holdStart = null;
  let ambiguityHoldSec = 0;
  let wrongSwitchCount = 0;
  let inFovFrames = 0;
  let frames = 0;

  for (let t = 0; t < durationSec; t += dt) {
    frames++;
    const pan = camera.yawDeg;
    const tilt = camera.tiltDeg;

    if (t - lastAnalysis >= analysisEverySec - 1e-9) {
      lastAnalysis = t;
      const { analysis } = analyzeAt(positions, truth, pan, tilt, { ...(cfg.renderOptions ?? {}), seed: seed + Math.round(t * 10) });
      lastAnalysisResult = analysis.direction;
      lastDecision = tracker.update(analysis.direction, { panDeg: pan, tiltDeg: tilt }, t);

      const truthDir = azElToUnit(truth.azDeg, truth.elDeg);
      const candWorld = analysis.direction.candidates.map((c) => {
        const w = azElToUnit(c.azDeg, c.elDeg);
        const y = pan * Math.PI / 180, tt = tilt * Math.PI / 180;
        const cy = Math.cos(y), sy = Math.sin(y), ct = Math.cos(tt), st = Math.sin(tt);
        const R = [[cy, -sy * st, sy * ct], [0, ct, st], [-sy, -cy * st, cy * ct]];
        return {
          azDeg: c.azDeg, elDeg: c.elDeg, deltaChi2: c.deltaChi2,
          world: [
            R[0][0] * w[0] + R[0][1] * w[1] + R[0][2] * w[2],
            R[1][0] * w[0] + R[1][1] * w[1] + R[1][2] * w[2],
            R[2][0] * w[0] + R[2][1] * w[1] + R[2][2] * w[2],
          ],
        };
      });

      events.push({
        t,
        pan, tilt,
        status: analysis.direction.status,
        candidateCount: analysis.direction.candidates.length,
        truthErrorDeg: analysis.direction.best
          ? angularSeparationDeg(
            rotateCandidatesToWorld(analysis.direction.best, pan, tilt),
            truthDir,
          )
          : null,
        decisionAction: lastDecision.action,
        consistent: tracker.consistentWorld() ? unitToAzEl(tracker.consistentWorld()) : null,
        candidatesWorld: candWorld.map((c) => ({ ...c, worldAzEl: unitToAzEl(c.world) })),
      });

      // Was the mirror family resolved and did the trusted bearing land on truth?
      const trusted = lastDecision?.bearing?.world ?? null;
      if (trusted) {
        const err = angularSeparationDeg(trusted, truthDir);
        finalTrustedError = err;
        if (mirrorResolvedAt == null && analysis.direction.status === 'resolved' && err < 15) {
          mirrorResolvedAt = t;
        }
      }
      if (lastDecision.action === 'hold' && lastDecision.reasonKey === 'ambiguous-hold') {
        holdStart = holdStart ?? t;
        ambiguityHoldSec += analysisEverySec;
      } else if (lastDecision.action === 'slew') {
        holdStart = null;
      }

      // Plan a probe when stuck on an ambiguity family.
      if (lastDecision.reasonKey === 'ambiguous-hold' && !probe && ambiguityHoldSec > (probeConfig.armAfterSec ?? 0.8)) {
        probePredicted = planProbeForDecision({
          decision: lastDecision,
          pose: { panDeg: pan, tiltDeg: tilt },
          planeNormalArray,
          probeDeg: probeConfig.probeDeg ?? 15,
        });
        if (probePredicted) {
          probe = probePredicted;
          probeStart = t;
        }
      }
    }

    // Execute a probe by slewing the camera to its target pose (sensor motion).
    let probeTarget = null;
    if (probe && !probeDone) {
      probeTarget = probe.newPose;
      const dPan = Math.abs(((probeTarget.yawDeg - pan + 540) % 360) - 180);
      const dTilt = Math.abs(probeTarget.tiltDeg - tilt);
      const cmd = {
        action: 'hold',
        reason: 'iprocin: reorienting to separate mirror branches',
        bearing: { world: null },
      };
      // camera.step with the probe target directly
      const stepRes = camera.step(probeTarget, dt);
      if (dPan < 1.5 && dTilt < 1.5) {
        probeDone = t;
        events.push({ t, probeDone: true, probe: { axis: probe.axis, deltaDeg: probe.deltaDeg, predictedSpurShiftDeg: probe.predictedSpurShiftDeg } });
      }
      if (stepRes.slewing) { /* counts as motion */ }
      // advance frame accounting
      const truthDirF = azElToUnit(truth.azDeg, truth.elDeg);
      if (camera.sees(truthDirF)) inFovFrames++;
      continue;
    }

    if (lastDecision) controller.command(lastDecision);
    controller.step(dt);
    const truthDir = azElToUnit(truth.azDeg, truth.elDeg);
    if (camera.sees(truthDir)) inFovFrames++;
  }

  return {
    events,
    metrics: {
      mirrorResolvedAtSec: mirrorResolvedAt,
      probeUsed: !!probe,
      probe: probe ? { axis: probe.axis, deltaDeg: probe.deltaDeg, predictedSpurShiftDeg: probe.predictedSpurShiftDeg, startSec: probeStart, doneSec: probeDone } : null,
      ambiguityHoldSec,
      lockRatio: frames ? inFovFrames / frames : 0,
      finalTrustedErrorDeg: finalTrustedError,
      finalPose: { pan: camera.yawDeg, tilt: camera.tiltDeg },
      finalTrusted: tracker.getConfirmedWorld(),
      lastStatus: lastAnalysisResult?.status ?? null,
    },
    tracker,
  };
}

function rotateCandidatesToWorld(c, pan, tilt) {
  const d = azElToUnit(c.azDeg, c.elDeg);
  const y = pan * Math.PI / 180, t = tilt * Math.PI / 180;
  const cy = Math.cos(y), sy = Math.sin(y), ct = Math.cos(t), st = Math.sin(t);
  const R = [[cy, -sy * st, sy * ct], [0, ct, st], [-sy, -cy * st, cy * ct]];
  return [
    R[0][0] * d[0] + R[0][1] * d[1] + R[0][2] * d[2],
    R[1][0] * d[0] + R[1][1] * d[1] + R[1][2] * d[2],
    R[2][0] * d[0] + R[2][1] * d[1] + R[2][2] * d[2],
  ];
}
