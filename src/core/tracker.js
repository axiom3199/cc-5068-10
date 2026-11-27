// Track-level logic between the per-frame direction solver and the virtual camera.
//
// Everything that persists across frames is stored in the WORLD frame. The solver
// reports array-frame angles; the tracker is handed the camera pose for the frame
// and converts each candidate with arrayToWorld(). This matters because the array
// is rigidly mounted to the camera: an array-frame angle that is "the same" in two
// frames points somewhere else in the world once the camera has paned.
//
// Responsibilities:
//   1. Keep DIRECTION HYPOTHESES (not one peak) across frames.
//   2. Maintain hypotheses through front/back ambiguity without pretending it is
//      resolved; merge them only when new observations separate them.
//   3. Decide slew-now vs hold-last-trusted, and never slew on a lone ambiguous
//      candidate.
//   4. Expose the current candidate set + credible ranges for rendering.

import {
  azElToUnit,
  angularSeparationDeg,
  unitToAzEl,
  norm,
  arrayToWorld,
} from './geometry.js';

const defaultConfig = {
  gateDeg: 15,             // association gate between a track and a new candidate
  maxHypotheses: 6,
  confirmFrames: 2,
  holdFrames: 10,
  decay: 0.86,
  mergeDeg: 7,
  pruneWeight: 0.08,
  ambiguityPairDeg: 22,    // candidates further apart than this form an ambiguity family
  excludeDeltaChi2: 30,    // a previously ambiguous branch this far behind is "excluded"
};

export class DirectionTracker {
  /**
   * @param {object} [config]
   * @param {{azDeg:number,elDeg:number}|null} [initialTrustedWorld] world az/el
   */
  constructor(config = {}, initialTrustedWorld = null) {
    this.config = { ...defaultConfig, ...config };
    this.tracks = [];
    this.nextId = 1;
    this.frameIndex = 0;
    this.lastTrustedWorld = initialTrustedWorld ? unitFromAzEl(initialTrustedWorld) : null;
    this.history = [];
    this.lastTime = null;
  }

  /**
   * Feed one per-frame solver result.
   * @param {object} result  analyzeDirection().direction
   * @param {object} pose    {panDeg, tiltDeg} camera pose for this frame
   * @param {number} [timeSec]
   */
  update(result, pose, timeSec = null) {
    this.frameIndex++;
    const cfg = this.config;
    const pan = pose?.panDeg ?? 0;
    const tilt = pose?.tiltDeg ?? 0;
    this.lastTime = timeSec ?? this.lastTime;

    const usable = result && result.best && (result.status === 'resolved' || result.status === 'ambiguous');
    const rawCandidates = usable ? result.candidates.filter((c) => c.plausible !== false) : [];
    // Feed the raw array-frame candidates to the cross-pose consistency check, which
    // is what lets a camera turn exclude one branch of a planar-array ambiguity.
    const consistent = this.observeForConsistency(rawCandidates, { panDeg: pan, tiltDeg: tilt });
    const candidates = rawCandidates.map((c) => ({
      ...c,
      world: arrayToWorld(azElToUnit(c.azDeg, c.elDeg), pan, tilt),
    }));

    for (const t of this.tracks) t.weight *= cfg.decay;

    const matched = new Set();
    for (const c of candidates) {
      let bestTrack = null;
      let bestScore = Infinity;
      for (const t of this.tracks) {
        if (matched.has(t.id)) continue;
        const sep = angularSeparationDeg(c.world, t.direction);
        if (sep > cfg.gateDeg) continue;
        const score = sep - 1.5 * Math.log(1 + t.weight);
        if (score < bestScore) { bestScore = score; bestTrack = t; }
      }
      if (bestTrack) {
        const alpha = Math.min(0.6, 0.25 + 0.08 * (bestTrack.hits ?? 1));
        bestTrack.direction = norm([
          bestTrack.direction[0] * (1 - alpha) + c.world[0] * alpha,
          bestTrack.direction[1] * (1 - alpha) + c.world[1] * alpha,
          bestTrack.direction[2] * (1 - alpha) + c.world[2] * alpha,
        ]);
        bestTrack.weight += c.plausible ? 1 : 0.35;
        bestTrack.hits = (bestTrack.hits ?? 1) + 1;
        bestTrack.misses = 0;
        bestTrack.lastChi2 = c.chi2;
        bestTrack.lastDeltaChi2 = c.deltaChi2;
        matched.add(bestTrack.id);
      } else {
        this.tracks.push({
          id: this.nextId++,
          direction: c.world,
          weight: c.plausible ? 1 : 0.4,
          hits: 1,
          misses: 0,
          bornFrame: this.frameIndex,
          lastChi2: c.chi2,
          lastDeltaChi2: c.deltaChi2,
        });
      }
    }
    for (const t of this.tracks) if (!matched.has(t.id)) t.misses = (t.misses ?? 0) + 1;

    // Merge tracks that have drifted together.
    this.tracks.sort((a, b) => b.weight - a.weight);
    const merged = [];
    for (const t of this.tracks) {
      const near = merged.find((m) => angularSeparationDeg(m.direction, t.direction) < cfg.mergeDeg);
      if (near) {
        const total = near.weight + t.weight;
        near.direction = norm([
          (near.direction[0] * near.weight + t.direction[0] * t.weight) / total,
          (near.direction[1] * near.weight + t.direction[1] * t.weight) / total,
          (near.direction[2] * near.weight + t.direction[2] * t.weight) / total,
        ]);
        near.weight = total;
        near.hits += t.hits;
        near.misses = Math.min(near.misses, t.misses);
      } else {
        merged.push(t);
      }
    }
    this.tracks = merged
      .filter((t) => t.weight >= cfg.pruneWeight && t.misses <= cfg.holdFrames)
      .slice(0, cfg.maxHypotheses);
    for (const t of this.tracks) {
      const ae = unitToAzEl(t.direction);
      t.azDeg = ae.azDeg;
      t.elDeg = ae.elDeg;
      t.confirmed = t.hits >= cfg.confirmFrames;
    }

    const decision = this.decide(result, candidates, consistent);
    this.history.push({
      frame: this.frameIndex,
      timeSec,
      action: decision.action,
      status: result?.status ?? 'none',
      bearing: decision.bearing,
      candidateCount: candidates.length,
      consistent: consistent ? unitToAzEl(consistent) : null,
    });
    if (this.history.length > 1200) this.history.shift();
    return decision;
  }

  decide(result, candidates, consistent = null) {
    const cfg = this.config;
    const status = result?.status ?? 'insufficient';
    const live = this.tracks.filter((t) => t.weight > 0.15).sort((a, b) => b.weight - a.weight);
    const confirmed = live.filter((t) => t.confirmed || t.hits >= cfg.confirmFrames);

    // A world bearing that has recurred across different camera poses is, by the
    // rigid-mount physics, the physical source. Adopt it if this frame's candidate
    // set still contains it (i.e. it was not contradicted by a newer observation).
    if (consistent && (status === 'resolved' || status === 'ambiguous')) {
      const stillSupported = candidates.some(
        (c) => c.plausible !== false && angularSeparationDeg(c.world, consistent) < cfg.gateDeg,
      );
      if (stillSupported) {
        this._pruneInconsistentTracks(consistent);
        return this.decideResolved(consistent, result);
      }
    }

    // A frame-level ambiguity family: two plausible candidates well apart. This is
    // the front/back mirror reported by the solver, and also two genuinely distinct
    // sources in the same frame.
    const families = [];
    for (let i = 0; i < candidates.length; i++) {
      for (let j = i + 1; j < candidates.length; j++) {
        if (candidates[i].plausible === false || candidates[j].plausible === false) continue;
        const sep = angularSeparationDeg(candidates[i].world, candidates[j].world);
        if (sep > cfg.ambiguityPairDeg) families.push({ a: candidates[i], b: candidates[j], sep });
      }
    }
    const consecutiveAmbiguity = confirmed.length >= 2
      && angularSeparationDeg(confirmed[0].direction, confirmed[1].direction) > cfg.ambiguityPairDeg;

    if (status === 'insufficient' && live.length === 0) {
      return this.holdDecision('insufficient-evidence', 'no reliable direction evidence in this frame; holding last trusted bearing');
    }

    if (families.length > 0 || consecutiveAmbiguity) {
      const fam = families[0];
      const family = consecutiveAmbiguity
        ? confirmed.slice(0, 2).map((t) => ({ azDeg: t.azDeg, elDeg: t.elDeg, world: t.direction }))
        : [fam.a, fam.b].map((c) => ({ azDeg: c.azDeg, elDeg: c.elDeg, world: c.world }));
      const chosen = this.pickForFov(family);
      // A fresh family with no consistency evidence yet still gets associated as
      // hypotheses so future frames can resolve it, but the camera must not move.
      return {
        action: 'hold',
        hold: true,
        reason: `ambiguous: ${family.length} candidate directions fit the TDOAs; holding ${this.lastTrustedWorld ? 'last trusted bearing' : 'current heading'}`,
        reasonKey: 'ambiguous-hold',
        ambiguity: {
          isAmbiguous: true,
          candidates: family.map((f) => ({ azDeg: f.azDeg, elDeg: f.elDeg, world: f.world })),
          separationDeg: family.length > 1 ? angularSeparationDeg(family[0].world, family[1].world) : null,
        },
        bearing: this.lastTrustedWorld
          ? { ...unitToAzEl(this.lastTrustedWorld), world: this.lastTrustedWorld, source: 'held-trusted' }
          : (chosen ? { azDeg: chosen.azDeg, elDeg: chosen.elDeg, world: chosen.world, source: 'candidate' } : null),
        tracks: this.tracks,
      };
    }

    if (status === 'resolved') {
      return this.decideResolved(live[0]?.direction ?? null, result);
    }

    return this.holdDecision(`status-${status}`, result?.reason ?? 'direction not usable this frame');
  }

  decideResolved(world, result) {
    const cfg = this.config;
    if (!world) return this.holdDecision('no-track', 'no candidate passed the tracking gate');
    const live = this.tracks.filter((t) => t.weight > 0.15).sort((a, b) => b.weight - a.weight);
    const target = live.find((t) => angularSeparationDeg(t.direction, world) < cfg.gateDeg) ?? live[0];
    if (!target) return this.holdDecision('no-track', 'no candidate passed the tracking gate');
    const nearTrusted = this.lastTrustedWorld
      && angularSeparationDeg(target.direction, this.lastTrustedWorld) < cfg.gateDeg;
    if (!target.confirmed && !nearTrusted) {
      return this.holdDecision('unconfirmed', 'new direction not yet confirmed by a second frame');
    }
    this.lastTrustedWorld = target.direction;
    const ae = unitToAzEl(target.direction);
    return {
      action: 'slew',
      hold: false,
      reason: nearTrusted && !target.confirmed ? 'continues last trusted bearing' : `direction confirmed across ${target.hits} frame(s)`,
      reasonKey: 'slew-confirmed',
      ambiguity: { isAmbiguous: false, candidates: [{ azDeg: ae.azDeg, elDeg: ae.elDeg, world: target.direction }] },
      bearing: { azDeg: ae.azDeg, elDeg: ae.elDeg, world: target.direction, source: 'tracked' },
      tracks: this.tracks,
    };
  }

  /**
   * Track consistency across camera motion. For a rigidly-mounted array, the TRUE
   * source is a fixed world direction; a mirror spur from a (near-)planar array is
   * a fixed ARRAY-frame angle, so it maps to a world direction that sweeps as the
   * camera pans/tilts. A hypothesis whose world direction recurs across poses with
   * different array-frame angles is the physical one; a hypothesis that only
   * recurs at a fixed array-frame angle is the geometric artefact.
   *
   * This method is called by the UI/driver with the raw candidates of each frame,
   * tagged with the camera pose. It returns the world bearing judged consistent, or
   * null when the evidence is still balanced.
   */
  consistentWorld() {
    return this._consistentWorld ?? null;
  }

  observeForConsistency(arrayCandidates, pose) {
    const pan = pose.panDeg;
    const tilt = pose.tiltDeg;
    const worldCands = arrayCandidates.map((c) => ({
      world: arrayToWorld(azElToUnit(c.azDeg, c.elDeg), pan, tilt),
      azDeg: c.azDeg,
      elDeg: c.elDeg,
      pose: { pan, tilt },
    }));
    this._consistencyBuffer = this._consistencyBuffer ?? [];
    // Replace, rather than append, when the pose is essentially unchanged: repeated
    // frames at one pose carry no cross-pose information and would otherwise bias
    // the comparison toward whichever branch is reported first.
    const samePose = this._consistencyBuffer.findIndex(
      (f) => Math.abs(f.pose.pan - pan) < 1 && Math.abs(f.pose.tilt - tilt) < 1,
    );
    const entry = { worldCands, pose: { pan, tilt } };
    if (samePose >= 0) this._consistencyBuffer[samePose] = entry;
    else this._consistencyBuffer.push(entry);
    if (this._consistencyBuffer.length > (this.config.consistencyFrames ?? 8)) {
      this._consistencyBuffer.shift();
    }
    if (!worldCands.length) return null;
    if (!this._hasPoseSpread()) { this._consistentWorld = null; return null; }

    // Score each candidate world direction by how many OTHER poses produced a
    // candidate landing within the gate. The physical source recurs at the same
    // world direction across poses; an array-frame mirror spur does not.
    const scores = worldCands.map((c) => {
      let distinctPoseSupport = 0;
      let maxCandSepAtDiffPose = 0;
      for (const frame of this._consistencyBuffer) {
        if (frame === entry) continue;
        if (Math.abs(frame.pose.pan - pan) < 1 && Math.abs(frame.pose.tilt - tilt) < 1) continue;
        let bestSep = Infinity;
        let matched = false;
        for (const x of frame.worldCands) {
          const sep = angularSeparationDeg(x.world, c.world);
          if (sep < bestSep) { bestSep = sep; matched = true; }
        }
        if (matched && bestSep < this.config.gateDeg) {
          distinctPoseSupport++;
          maxCandSepAtDiffPose = Math.max(maxCandSepAtDiffPose, bestSep);
        }
      }
      return { candidate: c, distinctPoseSupport, maxCandSepAtDiffPose };
    }).sort((a, b) => b.distinctPoseSupport - a.distinctPoseSupport);

    const top = scores[0];
    const opponent = scores.find((s) => angularSeparationDeg(s.candidate.world, top.candidate.world) > this.config.ambiguityPairDeg);
    if (!opponent) {
      if (top.distinctPoseSupport >= 1) { this._consistentWorld = top.candidate.world; return top.candidate.world; }
      this._consistentWorld = null;
      return null;
    }
    const margin = top.distinctPoseSupport - opponent.distinctPoseSupport;
    if (top.distinctPoseSupport >= 1 && margin >= 1) {
      this._consistentWorld = top.candidate.world;
      return top.candidate.world;
    }
    this._consistentWorld = null;
    return null;
  }

  _hasPoseSpread() {
    const pans = this._consistencyBuffer.map((f) => f.pose.pan);
    const tilts = this._consistencyBuffer.map((f) => f.pose.tilt);
    return (Math.max(...pans) - Math.min(...pans) > 5) || (Math.max(...tilts) - Math.min(...tilts) > 5);
  }


  pickForFov(family) {
    if (!this.lastTrustedWorld) return family[0];
    let best = family[0];
    let bd = Infinity;
    for (const f of family) {
      const d = angularSeparationDeg(this.lastTrustedWorld, f.world);
      if (d < bd) { bd = d; best = f; }
    }
    return best;
  }

  holdDecision(reasonKey, reason) {
    const ae = this.lastTrustedWorld ? unitToAzEl(this.lastTrustedWorld) : null;
    return {
      action: 'hold',
      hold: true,
      reason,
      reasonKey,
      ambiguity: { isAmbiguous: reasonKey === 'ambiguous-hold', candidates: [] },
      bearing: ae ? { azDeg: ae.azDeg, elDeg: ae.elDeg, world: this.lastTrustedWorld, source: 'held-trusted' } : null,
      tracks: this.tracks,
    };
  }

  /**
   * Called when a new observation clearly rejects one branch of a previous
   * ambiguity family: drop hypotheses that are now far behind the best fit.
   */
  pruneByEvidence(bestDeltaChi2ByTrackId) {
    this.tracks = this.tracks.filter((t) => {
      const d = bestDeltaChi2ByTrackId.get(t.id);
      return d == null || d < this.config.excludeDeltaChi2;
    });
  }

  /** Drop tracks that do not agree with a world bearing confirmed across poses. */
  _pruneInconsistentTracks(consistentWorld) {
    const before = this.tracks.length;
    this.tracks = this.tracks.filter(
      (t) => angularSeparationDeg(t.direction, consistentWorld) < this.config.gateDeg,
    );
    // Never let pruning remove every track: keep the closest survivor if needed.
    if (this.tracks.length === 0 && before > 0) {
      this.tracks = [{
        id: this.nextId++,
        direction: consistentWorld,
        weight: 1,
        hits: this.config.confirmFrames,
        misses: 0,
        confirmed: true,
        directionConfirmedByPose: true,
      }];
    }
    for (const t of this.tracks) {
      const ae = unitToAzEl(t.direction);
      t.azDeg = ae.azDeg;
      t.elDeg = ae.elDeg;
    }
  }

  getConfirmedWorld() {
    const c = this.tracks.filter((t) => t.confirmed).sort((a, b) => b.weight - a.weight);
    return c.length ? unitToAzEl(c[0].direction) : null;
  }
}

function unitFromAzEl({ azDeg, elDeg }) {
  return azElToUnit(azDeg, elDeg);
}
