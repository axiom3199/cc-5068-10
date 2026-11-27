// Geometry and coordinate conventions.
//
// World / array frame: x = right, y = up, z = forward (camera boresight at pan=tilt=0).
// Azimuth is measured from +z toward +x, i.e. positive = to the right. Elevation is
// measured from the horizontal plane, positive = up.
//
// Far-field source direction is the unit vector pointing FROM the array TOWARD the source.
// A microphone at p receives the wave earlier when p points toward the source, so the
// arrival-time offset of mic m relative to mic r is  tau_m = -(d · (p_m - p_r)) / c.
//
// Near-field uses the same sign convention with spherical arrival times:
//   tau_m = (|s - p_m| - |s - p_r|) / c,  s = distance * d.
//
// Camera pose (pan, tilt) is known to the system from its own encoders. The array is
// rigidly mounted to the camera, so world mic positions are R(pan,tilt) · p_local.

export const DEG = Math.PI / 180;

export function azElToUnit(azDeg, elDeg) {
  const az = azDeg * DEG;
  const el = Math.max(-89.999, Math.min(89.999, elDeg)) * DEG;
  const ce = Math.cos(el);
  return [Math.sin(az) * ce, Math.sin(el), Math.cos(az) * ce];
}

export function unitToAzEl(v) {
  const n = Math.hypot(v[0], v[1], v[2]) || 1;
  const x = v[0] / n;
  const y = Math.max(-1, Math.min(1, v[1] / n));
  const z = v[2] / n;
  return { azDeg: (Math.atan2(x, z) / DEG + 360) % 360, elDeg: Math.asin(y) / DEG };
}

export function norm(v) {
  const n = Math.hypot(v[0], v[1], v[2]);
  return n > 1e-12 ? [v[0] / n, v[1] / n, v[2] / n] : [0, 0, 1];
}

export function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function angularSeparationDeg(a, b) {
  return Math.acos(Math.max(-1, Math.min(1, dot(norm(a), norm(b))))) / DEG;
}

export function wrapAz(azDeg) {
  let a = azDeg % 360;
  if (a < 0) a += 360;
  return a;
}

export function azDiffDeg(a, b) {
  let d = ((a - b) % 360 + 540) % 360 - 180;
  return d;
}

// Ry(yaw) * Rx(-tilt): positive yaw turns right toward +x, positive tilt looks up.
export function rotationMatrix(yawDeg, tiltDeg) {
  const y = yawDeg * DEG;
  const t = tiltDeg * DEG;
  const cy = Math.cos(y), sy = Math.sin(y);
  const ct = Math.cos(t), st = Math.sin(t);
  return [
    [cy, -sy * st, sy * ct],
    [0, ct, st],
    [-sy, -cy * st, cy * ct],
  ];
}

export function rotateVec(R, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

export function rotatePoints(R, points) {
  return points.map((p) => rotateVec(R, p));
}

export function cameraForward(yawDeg, tiltDeg) {
  const y = yawDeg * DEG;
  const t = tiltDeg * DEG;
  return [Math.sin(y) * Math.cos(t), Math.sin(t), Math.cos(y) * Math.cos(t)];
}

export function maxBaseline(positions) {
  let m = 0;
  for (let i = 0; i < positions.length; i++) {
    for (let j = i + 1; j < positions.length; j++) {
      const d = Math.hypot(
        positions[i][0] - positions[j][0],
        positions[i][1] - positions[j][1],
        positions[i][2] - positions[j][2],
      );
      if (d > m) m = d;
    }
  }
  return m;
}

/**
 * Predicted arrival-time offsets (seconds) of every mic relative to the reference.
 * @param {object} o
 * @param {number[][]} o.positions local mic positions (m)
 * @param {number} o.referenceIndex
 * @param {number[]|{azDeg,elDeg}} o.direction unit vector or az/el object
 * @param {'far'|'near'} o.model
 * @param {number} o.distanceM array-origin distance, used by near model
 * @param {number} o.speedOfSound m/s
 */
export function predictedOffsets(o) {
  const { positions, referenceIndex, model, distanceM, speedOfSound } = o;
  const d = Array.isArray(o.direction) ? norm(o.direction) : azElToUnit(o.direction.azDeg, o.direction.elDeg);
  const ref = positions[referenceIndex];
  const out = new Array(positions.length);
  if (model === 'near') {
    const r = Math.max(0.01, distanceM);
    const s = [d[0] * r, d[1] * r, d[2] * r];
    const dist = positions.map((p) => Math.hypot(s[0] - p[0], s[1] - p[1], s[2] - p[2]));
    const dref = dist[referenceIndex];
    for (let m = 0; m < positions.length; m++) out[m] = (dist[m] - dref) / speedOfSound;
  } else {
    for (let m = 0; m < positions.length; m++) {
      out[m] = -(d[0] * (positions[m][0] - ref[0]) + d[1] * (positions[m][1] - ref[1]) + d[2] * (positions[m][2] - ref[2])) / speedOfSound;
    }
  }
  return out;
}

export function toCameraAngles(worldDirection, yawDeg, tiltDeg) {
  const R = rotationMatrix(yawDeg, tiltDeg);
  // d_cam = R^T * d_world
  const d = worldDirection;
  const local = [
    R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
    R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
    R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
  ];
  return { azDeg: Math.atan2(local[0], local[2]) / DEG, elDeg: Math.asin(Math.max(-1, Math.min(1, local[1]))) / DEG };
}

export function inFov(directionWorld, yawDeg, tiltDeg, hFovDeg, vFovDeg) {
  const a = toCameraAngles(directionWorld, yawDeg, tiltDeg);
  return Math.abs(a.azDeg) <= hFovDeg / 2 && Math.abs(a.elDeg) <= vFovDeg / 2;
}

// Slew a unit direction toward a target with a maximum angular rate.
export function slewDirection(current, target, maxDegPerSec, dtSec) {
  const c = norm(current);
  const t = norm(target);
  const sep = angularSeparationDeg(c, t);
  if (sep < 1e-9) return { direction: t, movedDeg: 0, remainingDeg: 0 };
  const step = Math.min(sep, maxDegPerSec * dtSec);
  const a = (step / sep) * DEG;
  const s = Math.sin(a);
  const mixed = [0, 0, 0];
  // normalized linear interpolation is a good approximation for small steps and
  // is exact in direction; use a proper slerp to avoid speed shrinkage.
  if (sep > 1e-6) {
    const wa = Math.sin((1 - step / sep) * (sep * DEG)) / Math.sin(sep * DEG);
    const wb = Math.sin((step / sep) * (sep * DEG)) / Math.sin(sep * DEG);
    mixed[0] = c[0] * wa + t[0] * wb;
    mixed[1] = c[1] * wa + t[1] * wb;
    mixed[2] = c[2] * wa + t[2] * wb;
  }
  return { direction: norm(mixed), movedDeg: step, remainingDeg: sep - step };
}

// ---------------------------------------------------------------------------
// Array frame <-> world frame
//
// In this project the microphone array is rigidly attached to the camera, so the
// mapping between the two frames is the camera pose itself:
//   d_world = R(pan, tilt) · d_array
// Inference always runs in the array frame (that is where mic positions live).
// Anything that persists across camera motion - the tracker, the last trusted
// bearing, the camera target - must be kept in the WORLD frame, because a fixed
// array-frame angle describes a different world direction after the camera turns.

export function arrayToWorld(direction, panDeg, tiltDeg) {
  return norm(rotateVec(rotationMatrix(panDeg, tiltDeg), norm(direction)));
}

export function worldToArray(direction, panDeg, tiltDeg) {
  const R = rotationMatrix(panDeg, tiltDeg);
  const d = norm(direction);
  return norm([
    R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
    R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
    R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
  ]);
}

/** Convert an array-frame az/el observation into world az/el. */
export function arrayAzElToWorld(azDeg, elDeg, panDeg, tiltDeg) {
  return unitToAzEl(arrayToWorld(azElToUnit(azDeg, elDeg), panDeg, tiltDeg));
}

/** Convert a world az/el observation into array-frame az/el. */
export function worldAzElToArray(azDeg, elDeg, panDeg, tiltDeg) {
  return unitToAzEl(worldToArray(azElToUnit(azDeg, elDeg), panDeg, tiltDeg));
}
