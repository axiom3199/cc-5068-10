// Virtual camera with a known pan/tilt encoder and a nano slew limit.
//
// The camera is a separate component from the inference core: the tracker reports
// bearings in the WORLD frame, and the controller decides whether to slew or hold.
// The evaluation harness uses a synthetic "locked object" (the true source) to
// score how long it stays inside the camera field of view, independent of the
// reported peak angle.

import { angularSeparationDeg, toCameraAngles, norm, DEG } from './geometry.js';

export const CAMERA_DEFAULTS = Object.freeze({
  hFovDeg: 55,
  vFovDeg: 34,
  maxPanDegPerSec: 70,
  maxTiltDegPerSec: 50,
  settleDeg: 0.35,
  deadbandDeg: 0.8,
  minDwellSec: 0.14,
});

export class VirtualCamera {
  constructor(config = {}, pose = {}) {
    this.config = { ...CAMERA_DEFAULTS, ...config };
    this.yawDeg = pose.yawDeg ?? 0;
    this.tiltDeg = pose.tiltDeg ?? 0;
    this.velocityDps = { pan: 0, tilt: 0 };
    this.slewing = false;
    this.target = null;
  }

  /** World-frame forward vector of the camera. */
  get forward() {
    const y = this.yawDeg * DEG;
    const t = this.tiltDeg * DEG;
    return [Math.sin(y) * Math.cos(t), Math.sin(t), Math.cos(y) * Math.cos(t)];
  }

  /**
   * Advance the camera toward a target pan/tilt with rate limits.
   * @param {{yawDeg:number,tiltDeg:number}|null} target null = hold current pose
   * @param {number} dtSec
   * @returns {{slewing:boolean, movedDeg:number, settled:boolean}}
   */
  step(target, dtSec) {
    const cfg = this.config;
    const dt = Math.max(1e-4, Math.min(0.2, dtSec ?? 1 / 60));
    this.target = target;
    if (!target) {
      this.velocityDps.pan *= 0.6;
      this.velocityDps.tilt *= 0.6;
      this.slewing = false;
      return { slewing: false, movedDeg: 0, settled: true };
    }
    const dYaw = ((target.yawDeg - this.yawDeg + 540) % 360) - 180;
    const dTilt = Math.max(-160, Math.min(160, target.tiltDeg - this.tiltDeg));
    let stepYaw = 0;
    let stepTilt = 0;
    const maxYaw = cfg.maxPanDegPerSec * dt;
    const maxTilt = cfg.maxTiltDegPerSec * dt;
    if (Math.abs(dYaw) > cfg.deadbandDeg) {
      stepYaw = Math.max(-maxYaw, Math.min(maxYaw, dYaw));
      if (Math.abs(dYaw - stepYaw) < cfg.deadbandDeg) stepYaw = dYaw;
    }
    if (Math.abs(dTilt) > cfg.deadbandDeg) {
      stepTilt = Math.max(-maxTilt, Math.min(maxTilt, dTilt));
      if (Math.abs(dTilt - stepTilt) < cfg.deadbandDeg) stepTilt = dTilt;
    }
    this.yawDeg = ((this.yawDeg + stepYaw) % 360 + 360) % 360;
    this.tiltDeg = Math.max(-80, Math.min(80, this.tiltDeg + stepTilt));
    const speed = Math.hypot(stepYaw, stepTilt) / dt;
    this.velocityDps = { pan: stepYaw / dt, tilt: stepTilt / dt };
    this.slewing = speed > 1;
    const remaining = Math.hypot(dYaw - stepYaw, dTilt - stepTilt);
    return { slewing: this.slewing, movedDeg: Math.hypot(stepYaw, stepTilt), settled: remaining < cfg.settleDeg };
  }

  /**
   * Is a world direction inside the current field of view?
   * @param {number[]} worldDirection
   * @param {number} [marginDeg] shrink the effective FOV by this margin
   */
  sees(worldDirection, marginDeg = 0) {
    const a = toCameraAngles(worldDirection, this.yawDeg, this.tiltDeg);
    return Math.abs(a.azDeg) <= this.config.hFovDeg / 2 - marginDeg
      && Math.abs(a.elDeg) <= this.config.vFovDeg / 2 - marginDeg;
  }

  /** Pixel-normalized offset of a world direction, for drawing a reticle. */
  project(worldDirection) {
    const a = toCameraAngles(worldDirection, this.yawDeg, this.tiltDeg);
    const u = a.azDeg / (this.config.hFovDeg / 2);
    const v = a.elDeg / (this.config.vFovDeg / 2);
    return { u, v, inFov: Math.abs(u) <= 1 && Math.abs(v) <= 1, azDeg: a.azDeg, elDeg: a.elDeg };
  }
}

/**
 * Camera control policy driven purely by the tracker decision.
 * - slew: aim at the confirmed bearing.
 * - hold: keep the previous aim (the last trusted bearing) - never follow a lone
 *   ambiguous candidate.
 */
export class CameraController {
  constructor(camera, config = {}) {
    this.camera = camera;
    this.config = { ...CAMERA_DEFAULTS, ...config };
    this.commandedWorld = null;
    this.lastCommand = { action: 'hold', reason: 'startup' };
    this.everSlewed = false;
  }

  /** @param {object} decision tracker output (bearing is world-frame) */
  command(decision) {
    if (!decision?.bearing?.world) {
      this.lastCommand = { action: 'hold', reason: decision?.reason ?? 'no bearing', world: null };
      return this.lastCommand;
    }
    const world = norm(decision.bearing.world);
    if (decision.action === 'slew') {
      this.commandedWorld = world;
      this.everSlewed = true;
      this.lastCommand = { action: 'slew', reason: decision.reason, world };
    } else {
      this.lastCommand = { action: 'hold', reason: decision.reason, world: this.commandedWorld };
    }
    return this.lastCommand;
  }

  step(dtSec) {
    let target = null;
    if (this.lastCommand.action === 'slew' && this.commandedWorld) {
      const [x, y, z] = this.commandedWorld;
      const yawDeg = (Math.atan2(x, z) / DEG + 360) % 360;
      const tiltDeg = Math.asin(Math.max(-1, Math.min(1, y))) / DEG;
      target = { yawDeg, tiltDeg };
    }
    return this.camera.step(target, dtSec);
  }
}

export function worldAngleBetween(a, b) {
  return angularSeparationDeg(a, b);
}

