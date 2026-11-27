// Disambiguation by deliberate array reorientation ("ambiguity probe").
//
// A rigid array mounted on the camera has a fixed array-frame geometry. For a
// (near-)planar array the direction solver returns a mirror pair: the true array-
// frame direction d_a and its reflection d_m across the array plane. When the rig
// rotates by R, the physical source's world direction stays fixed, so the measured
// array-frame direction becomes Rᵀ·d_world, and the mirror becomes reflect(Rᵀ·d_world).
// The two WORLD candidates are therefore:
//     truth  : d_world                      (constant by definition)
//     spur   : R · reflect(Rᵀ · d_world)     (sweeps as R changes, unless the
//                                             rotation axis is parallel to the
//                                             plane normal)
// Rotating about an axis parallel to the plane normal leaves the spur fixed, which
// is why panning a HORIZONTAL array does not help (its normal is vertical) but
// tilting does; for a VERTICAL array the reverse holds. This module predicts which
// motion actually separates the spur, so the probe is informative rather than blind.

import {
  azElToUnit,
  unitToAzEl,
  angularSeparationDeg,
  rotationMatrix,
  norm,
} from './geometry.js';

const DEG = Math.PI / 180;

function rotate(R, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

function rotateT(R, v) {
  return [
    R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
    R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
    R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2],
  ];
}

export function reflectAcrossPlane(d, n) {
  const nn = norm(n);
  const dot = d[0] * nn[0] + d[1] * nn[1] + d[2] * nn[2];
  return norm([d[0] - 2 * dot * nn[0], d[1] - 2 * dot * nn[1], d[2] - 2 * dot * nn[2]]);
}

/**
 * World direction of the mirror spur at a given camera pose, given a world-frame
 * hypothesis for the source and the array plane normal in the array frame.
 */
export function mirrorWorldAt(worldSource, pose, planeNormalArray) {
  const R = rotationMatrix(pose.yawDeg ?? pose.panDeg ?? 0, pose.tiltDeg ?? 0);
  const dArray = rotateT(R, worldSource);
  const mirrorArray = reflectAcrossPlane(dArray, planeNormalArray);
  return norm(rotate(R, mirrorArray));
}

/**
 * Choose the reorientation that best separates the spur from the fixed source.
 * Tries ±pan and ±tilt; returns the axis/sign with the largest predicted spur
 * movement. The "source" world direction is assumed fixed, so its predicted
 * movement is zero by construction; the score is the spur displacement.
 *
 * @param {object} o
 * @param {number[]} o.worldSource          current best hypothesis, world frame
 * @param {{panDeg?:number,tiltDeg?:number,yawDeg?:number}} o.pose
 * @param {number[]} o.planeNormalArray     array plane normal in array frame
 * @param {number} [o.probeDeg=15]
 * @returns {{axis:'pan'|'tilt', deltaDeg:number, predictedSpurShiftDeg:number,
 *            currentSpur:number[], predictedSpur:number[]}|null}
 */
export function recommendProbe(o) {
  const { worldSource, pose, planeNormalArray } = o;
  const probeDeg = o.probeDeg ?? 15;
  const pan = pose.panDeg ?? pose.yawDeg ?? 0;
  const tilt = pose.tiltDeg ?? 0;
  const currentSpur = mirrorWorldAt(worldSource, { yawDeg: pan, tiltDeg: tilt }, planeNormalArray);
  const candidates = [];
  for (const axis of ['pan', 'tilt']) {
    for (const sign of [1, -1]) {
      const newPose = axis === 'pan'
        ? { yawDeg: pan + sign * probeDeg, tiltDeg: tilt }
        : { yawDeg: pan, tiltDeg: Math.max(-80, Math.min(80, tilt + sign * probeDeg)) };
      const predictedSpur = mirrorWorldAt(worldSource, newPose, planeNormalArray);
      const shift = angularSeparationDeg(currentSpur, predictedSpur);
      // Prefer the axis whose motion stays inside a comfortable tilt range and
      // whose predicted shift is largest.
      candidates.push({ axis, sign, deltaDeg: sign * probeDeg, newPose, predictedSpur, shift });
    }
  }
  candidates.sort((a, b) => b.shift - a.shift);
  const best = candidates[0];
  if (!best || best.shift < (o.minUsefulShiftDeg ?? 3)) return null;
  return {
    axis: best.axis,
    deltaDeg: best.deltaDeg,
    newPose: best.newPose,
    predictedSpurShiftDeg: best.shift,
    currentSpur,
    predictedSpur: best.predictedSpur,
    alternatives: candidates,
  };
}

/**
 * Decide, from a tracker decision and the current candidate set, whether a probe
 * would help. Returns null when the situation is not an ambiguity family or when
 * the last trusted bearing should simply be held instead.
 */
export function planProbeForDecision({ decision, pose, planeNormalArray, probeDeg = 15, worldSource = null }) {
  if (!decision?.ambiguity?.isAmbiguous) return null;
  const fam = decision.ambiguity.candidates ?? [];
  if (fam.length < 2) return null;
  // Use the world direction of the closest candidate to the held bearing (or the
  // first) as the working "source" hypothesis; the two branches are mirror-symmetric
  // so the probe recommendation is the same for either.
  const src = worldSource ?? fam[0].world
    ?? azElToUnit(fam[0].azDeg, fam[0].elDeg);
  return recommendProbe({ worldSource: src, pose, planeNormalArray, probeDeg });
}

export function worldAzEl(direction) {
  return unitToAzEl(direction);
}

export { angularSeparationDeg, DEG };
