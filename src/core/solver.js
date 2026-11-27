// Direction solver: fits measured pair TDOAs to known array geometry.
//
// The solver never sees how the audio was produced. Inputs are channel positions
// (array frame), measured pair lags with per-pair uncertainty, sample rate,
// speed of sound and the array-frame reference channel.
//
// It returns candidate directions plus explicit credible ranges. Geometric
// ambiguities are treated as first-class results, not forced into a single angle:
//   - planar array: mirror solution reflected across the array plane (front/back
//     or up/down), exact for ideal planar geometry;
//   - collinear array: a full cone of directions with the same axial angle;
//   - small/near-planar array: multiple shallow minima that may be separated by
//     later observations.
//
// Angle conventions match geometry.js: azimuth from +z toward +x, elevation up,
// direction vector points from the array toward the source.

import {
  azElToUnit,
  unitToAzEl,
  angularSeparationDeg,
  predictedOffsets,
  norm,
  DEG,
} from './geometry.js';

const clampEl = (v) => Math.max(-89, Math.min(89, v));
const wrapAz = (v) => ((v % 360) + 360) % 360;

// ---------------------------------------------------------------- linear algebra

function jacobiEigen3(a0) {
  const a = a0.map((r) => r.slice());
  const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 50; sweep++) {
    let off = 0;
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) off += a[p][q] * a[p][q];
    if (off < 1e-30) break;
    for (let p = 0; p < 3; p++) {
      for (let q = p + 1; q < 3; q++) {
        if (Math.abs(a[p][q]) < 1e-300) continue;
        const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 3; k++) {
          const akp = a[k][p];
          const akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = a[p][k];
          const aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = v[k][p];
          const vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const order = [0, 1, 2].sort((i, j) => a[j][j] - a[i][i]);
  return {
    values: order.map((i) => a[i][i]),
    vectors: order.map((i) => [v[0][i], v[1][i], v[2][i]]),
  };
}

function invertSym3(m) {
  const [a, b, c] = m[0];
  const [d, e, f] = m[1];
  const [g, h, i] = m[2];
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (!Number.isFinite(det) || Math.abs(det) < 1e-24) return null;
  return [
    [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
    [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
    [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
  ];
}

// ------------------------------------------------------------ array geometry

/**
 * Characterize the microphone layout.
 * @param {number[][]} positions
 * @param {{planarityTolM?:number, collinearTolM?:number}} [opts]
 */
export function analyzeArrayGeometry(positions, opts = {}) {
  const n = positions.length;
  const planarityTol = opts.planarityTolM ?? 1e-3;
  const collinearTol = opts.collinearTolM ?? 1e-3;
  if (n === 0) return { rank: 0, kind: 'none', points: [] };
  const centroid = [0, 0, 0];
  for (const p of positions) for (let k = 0; k < 3; k++) centroid[k] += p[k] / n;
  const centered = positions.map((p) => p.map((v, k) => v - centroid[k]));
  const cov = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const p of centered) {
    for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) cov[a][b] += (p[a] * p[b]) / n;
  }
  const { values, vectors } = jacobiEigen3(cov);
  const sd = values.map((v) => Math.sqrt(Math.max(0, v)));
  let maxSpan = 0;
  for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) {
    maxSpan = Math.max(maxSpan, Math.hypot(...positions[i].map((v, k) => v - positions[j][k])));
  }
  let kind = '3d';
  if (sd[1] < collinearTol) kind = 'collinear';
  else if (sd[2] < planarityTol) kind = 'planar';
  // A second/third dimension well below the first is effectively degenerate even
  // when numerically non-zero.
  if (kind === '3d' && sd[1] < Math.max(collinearTol, 0.02 * sd[0])) {
    kind = 'near-collinear';
  }
  if (kind === '3d' && sd[2] < Math.max(planarityTol, 0.02 * sd[0])) {
    kind = 'near-planar';
  }
  return {
    points: positions.length,
    centroid,
    centered,
    covariance: cov,
    singularValues: sd,
    outOfPlaneRmsM: sd[2],
    outOfLineRmsM: sd[1],
    maxSpanM: maxSpan,
    rank: kind === 'collinear' ? 1 : kind === 'planar' ? 2 : 3,
    kind,
    axis: norm(vectors[0]),
    planeNormal: norm(vectors[2]),
  };
}

// ------------------------------------------------------------- residual model

function makeResidualModel({ observations, positions, referenceIndex, speedOfSound, model, distanceM }) {
  return function residual(params) {
    const azDeg = params[0];
    const elDeg = params[1];
    const r = params.length > 2 ? Math.exp(params[2]) : distanceM;
    const taus = predictedOffsets({
      positions,
      referenceIndex,
      direction: { azDeg, elDeg },
      model,
      distanceM: r,
      speedOfSound,
    });
    let chi2 = 0;
    const res = new Float64Array(observations.length);
    for (let i = 0; i < observations.length; i++) {
      const o = observations[i];
      const e = taus[o.pair[0]] - taus[o.pair[1]] - o.tau;
      res[i] = e;
      const z = e / o.sigma;
      chi2 += z * z;
    }
    return { chi2, res, taus };
  };
}

function nelderMead(f, x0, opts = {}) {
  const n = x0.length;
  const step = opts.step ?? x0.map(() => 2);
  let simplex = [x0.slice()];
  for (let i = 0; i < n; i++) {
    const p = x0.slice();
    p[i] += step[i] ?? 2;
    simplex.push(p);
  }
  let values = simplex.map((p) => f(p).chi2);
  const maxIter = opts.maxIter ?? 250;
  const tol = opts.tol ?? 1e-10;
  for (let iter = 0; iter < maxIter; iter++) {
    const order = values.map((_, i) => i).sort((a, b) => values[a] - values[b]);
    simplex = order.map((i) => simplex[i]);
    values = order.map((i) => values[i]);
    if (Math.abs(values[n] - values[0]) < tol * (Math.abs(values[0]) + 1e-12)) break;
    const centroid = new Array(n).fill(0);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) centroid[j] += simplex[i][j] / n;
    const worst = simplex[n];
    const reflected = centroid.map((c, j) => c + (c - worst[j]));
    const fr = f(reflected);
    if (fr.chi2 < values[0]) {
      const expanded = centroid.map((c, j) => c + 2 * (c - worst[j]));
      const fe = f(expanded);
      if (fe.chi2 < fr.chi2) { simplex[n] = expanded; values[n] = fe.chi2; }
      else { simplex[n] = reflected; values[n] = fr.chi2; }
    } else if (fr.chi2 < values[n - 1]) {
      simplex[n] = reflected;
      values[n] = fr.chi2;
    } else {
      const contracted = centroid.map((c, j) => c + 0.5 * (worst[j] - c));
      const fc = f(contracted);
      if (fc.chi2 < values[n]) { simplex[n] = contracted; values[n] = fc.chi2; }
      else {
        for (let i = 1; i <= n; i++) {
          simplex[i] = simplex[i].map((v, j) => simplex[0][j] + 0.5 * (v - simplex[0][j]));
          values[i] = f(simplex[i]).chi2;
        }
      }
    }
  }
  const best = values.indexOf(Math.min(...values));
  return { x: simplex[best], value: values[best] };
}

function numericJacobian(params, model, observations) {
  const base = 1e-4;
  const J = observations.map(() => new Float64Array(params.length));
  for (let p = 0; p < params.length; p++) {
    const up = params.slice();
    const dn = params.slice();
    up[p] += base;
    dn[p] -= base;
    const ru = model(up);
    const rd = model(dn);
    for (let i = 0; i < observations.length; i++) {
      const du = ru.taus[observations[i].pair[0]] - ru.taus[observations[i].pair[1]];
      const dd = rd.taus[observations[i].pair[0]] - rd.taus[observations[i].pair[1]];
      J[i][p] = (du - dd) / (2 * base);
    }
  }
  return J;
}

/**
 * Fit at a start point with Huber IRLS down-weighting, returning covariance and
 * residual diagnostics. Parameters are [azDeg, elDeg] or [azDeg, elDeg, ln r].
 */
export function fitAt(params0, cfg) {
  const { observations, positions, referenceIndex, speedOfSound, model } = cfg;
  const isNear = model === 'near';
  let obs = observations.map((o) => ({ ...o }));
  let fit = null;
  let modelFn = null;
  const start = params0.slice();
  start[0] = wrapAz(start[0]);
  start[1] = clampEl(start[1]);
  if (isNear) start[2] = Math.max(Math.log(0.1), Math.min(Math.log(30), start[2]));

  for (let iter = 0; iter < 5; iter++) {
    modelFn = makeResidualModel({ observations: obs, positions, referenceIndex, speedOfSound, model, distanceM: null });
    const f = (p) => {
      const q = p.slice();
      q[0] = wrapAz(q[0]);
      q[1] = clampEl(q[1]);
      if (q.length > 2) q[2] = Math.max(Math.log(0.1), Math.min(Math.log(30), q[2]));
      return modelFn(q);
    };
    const step = isNear ? [3, 2, 0.2] : [2.5, 2.5];
    const nm = nelderMead(f, start.slice(), { step, maxIter: 260, tol: 1e-11 });
    const p = nm.x.slice();
    p[0] = wrapAz(p[0]);
    p[1] = clampEl(p[1]);
    if (isNear) p[2] = Math.max(Math.log(0.1), Math.min(Math.log(30), p[2]));
    start.splice(0, start.length, ...p);
    const ev = modelFn(p);
    fit = { params: p, chi2: ev.chi2, res: ev.res, taus: ev.taus };

    const dof = Math.max(1, obs.length - (isNear ? 3 : 2));
    const scale = Math.sqrt(ev.chi2 / dof);
    let changed = false;
    const next = observations.map((o, i) => {
      const z = Math.abs(ev.res[i]) / (o.sigma * Math.max(1, scale));
      if (z > 1.5) {
        changed = true;
        return { ...o, sigma: o.sigma * Math.min(4, z / 1.5) };
      }
      return o;
    });
    if (!changed) break;
    obs = next;
  }

  const dof = Math.max(1, observations.length - (isNear ? 3 : 2));
  const chi2Red = fit.chi2 / dof;
  const J = numericJacobian(fit.params, modelFn, observations);
  const H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < observations.length; i++) {
    const w = 1 / (observations[i].sigma * observations[i].sigma);
    for (let a = 0; a < fit.params.length; a++) {
      for (let b = 0; b < fit.params.length; b++) H[a][b] += w * J[i][a] * J[i][b];
    }
  }
  const scaleVar = Math.max(1, chi2Red);
  const dim = fit.params.length;
  let cov = null;
  if (dim === 2) {
    const det = H[0][0] * H[1][1] - H[0][1] * H[1][0];
    if (Number.isFinite(det) && Math.abs(det) > 1e-30) {
      cov = [[H[1][1] / det, -H[0][1] / det], [-H[1][0] / det, H[0][0] / det]];
    }
  } else {
    cov = invertSym3(H);
  }
  const sigmaAzDeg = cov ? Math.sqrt(Math.max(0, cov[0][0] * scaleVar)) : Infinity;
  const sigmaElDeg = cov ? Math.sqrt(Math.max(0, cov[1][1] * scaleVar)) : Infinity;
  const sigmaDistLog = isNear && cov && cov[2] ? Math.sqrt(Math.max(0, cov[2][2] * scaleVar)) : null;

  const azDeg = wrapAz(fit.params[0]);
  const elDeg = fit.params[1];
  return {
    azDeg,
    elDeg,
    distanceM: isNear ? Math.exp(fit.params[2]) : null,
    direction: azElToUnit(azDeg, elDeg),
    chi2: fit.chi2,
    dof,
    chi2Red,
    sigmaAzDeg,
    sigmaElDeg,
    sigmaDistanceM: isNear && sigmaDistLog != null ? Math.exp(fit.params[2]) * sigmaDistLog : null,
    cov3: dim === 3 ? cov : null,
    params: fit.params,
    residuals: Array.from(fit.res),
    nearBoundHit: isNear ? (fit.params[2] <= Math.log(0.101) || fit.params[2] >= Math.log(29.9)) : false,
    observations: observations.map((o, i) => ({
      pair: o.pair,
      tauObs: o.tau,
      tauPred: fit.taus[o.pair[0]] - fit.taus[o.pair[1]],
      sigma: o.sigma,
      residual: fit.res[i],
      coherence: o.coherence,
      peakToFloor: o.peakToFloor,
    })),
    tausPredicted: Array.from(fit.taus),
  };
}

// ---------------------------------------------------------------------- grid

function makeGrid(cfg) {
  const { observations, positions, referenceIndex, speedOfSound, model } = cfg;
  const azStep = cfg.grid?.azStep ?? 4;
  const elStep = cfg.grid?.elStep ?? 4;
  const elMin = cfg.grid?.elMin ?? -84;
  const elMax = cfg.grid?.elMax ?? 84;
  const distances = model === 'near'
    ? (cfg.grid?.distancesM ?? [0.3, 0.45, 0.65, 0.9, 1.3, 1.8, 2.5, 3.5, 5])
    : null;
  const modelFn = makeResidualModel({
    observations,
    positions,
    referenceIndex,
    speedOfSound,
    model,
    distanceM: null,
  });
  const points = [];
  for (let el = elMin; el <= elMax; el += elStep) {
    for (let az = 0; az < 360; az += azStep) {
      let best = Infinity;
      if (distances) {
        for (const d of distances) best = Math.min(best, modelFn([az, el, Math.log(d)]).chi2);
      } else {
        best = modelFn([az, el]).chi2;
      }
      points.push({ azDeg: az, elDeg: el, chi2: best });
    }
  }
  return { points, azStep, elStep, azs: [...new Set(points.map((p) => p.azDeg))], els: [...new Set(points.map((p) => p.elDeg))] };
}

function localMinima(grid, points) {
  const { azStep, elStep } = grid;
  const map = new Map(points.map((p) => [`${p.azDeg}|${p.elDeg}`, p]));
  const mins = [];
  for (const p of points) {
    let isMin = true;
    for (let dz = -1; dz <= 1 && isMin; dz++) {
      for (let de = -1; de <= 1; de++) {
        if (!dz && !de) continue;
        const q = map.get(`${wrapAz(p.azDeg + dz * azStep)}|${p.elDeg + de * elStep}`);
        if (q && q.chi2 < p.chi2 - 1e-9) { isMin = false; break; }
      }
    }
    if (isMin) mins.push(p);
  }
  return mins.sort((a, b) => a.chi2 - b.chi2);
}

function dedupeFits(fits, minSepDeg = 9) {
  const sorted = [...fits].sort((a, b) => a.chi2 - b.chi2);
  const kept = [];
  for (const f of sorted) {
    if (kept.some((k) => angularSeparationDeg(k.direction, f.direction) < minSepDeg)) continue;
    kept.push(f);
  }
  return kept;
}

function fitCandidateSet(model, good, cfg, grid) {
  const baseCfg = {
    observations: good,
    positions: cfg.positions,
    referenceIndex: cfg.referenceIndex,
    speedOfSound: cfg.speedOfSound,
    model,
    grid: cfg.grid,
  };
  const minima = localMinima(grid, grid.points).slice(0, 10);
  const starts = dedupeFits(
    minima.map((m) => ({
      chi2: m.chi2,
      direction: azElToUnit(m.azDeg, m.elDeg),
      params: model === 'near' ? [m.azDeg, m.elDeg, Math.log(cfg.distanceM ?? 1)] : [m.azDeg, m.elDeg],
    })),
    12,
  );
  if (starts.length === 0) starts.push({ params: model === 'near' ? [0, 0, Math.log(1)] : [0, 0] });
  const fits = [];
  for (const s of starts) {
    try {
      fits.push(fitAt(s.params, baseCfg));
    } catch (e) {
      /* skip failed start */
    }
  }
  return dedupeFits(fits, 8);
}

// -------------------------------------------------------- credible regions

function profileRegion(fits, observations, cfg, grid, deltaChi2) {
  const best = fits[0];
  const azs = [];
  const els = [];
  const dists = [];
  const cands = fits.map((f) => f.direction);
  const nearBest = (p) => {
    let bi = 0;
    let bd = Infinity;
    const pd = azElToUnit(p.azDeg, p.elDeg);
    cands.forEach((c, i) => {
      const d = angularSeparationDeg(pd, c);
      if (d < bd) { bd = d; bi = i; }
    });
    return { index: bi, sep: bd };
  };
  let bestAz = best.azDeg;
  let bestEl = best.elDeg;
  for (let el = Math.max(-89, bestEl - 12); el <= Math.min(89, bestEl + 12); el += 1) {
    for (let az = bestAz - 12; az <= bestAz + 12; az += 1) {
      const azW = wrapAz(az);
      const modelFn = makeResidualModel({
        observations,
        positions: cfg.positions,
        referenceIndex: cfg.referenceIndex,
        speedOfSound: cfg.speedOfSound,
        model: cfg.model,
        distanceM: best.distanceM ?? cfg.distanceM,
      });
      let chi2;
      if (cfg.model === 'near') {
        chi2 = Infinity;
        const d0 = Math.exp(fits[0].params[2]);
        for (const dd of [d0 * 0.7, d0, d0 * 1.4]) chi2 = Math.min(chi2, modelFn([azW, el, Math.log(dd)]).chi2);
      } else {
        chi2 = modelFn([azW, el]).chi2;
      }
      if (chi2 <= best.chi2 + deltaChi2) {
        const { index } = nearBest({ azDeg: azW, elDeg: el });
        if (index === 0) {
          azs.push(bestAz + (((azW - bestAz + 540) % 360) - 180));
          els.push(el);
        }
      }
    }
  }
  if (azs.length === 0) return null;
  const azMin = Math.min(...azs);
  const azMax = Math.max(...azs);
  const elMin = Math.min(...els);
  const elMax = Math.max(...els);
  return {
    azRangeDeg: [wrapAz(azMin), wrapAz(azMax)],
    azExtentDeg: azMax - azMin,
    elRangeDeg: [elMin, elMax],
    elExtentDeg: elMax - elMin,
    deltaChi2,
    sampledPoints: azs.length,
  };
}

// ------------------------------------------------------------- main solver

function insufficient(reason, extra = {}) {
  return {
    status: 'insufficient',
    reason,
    confidence: 0,
    evidence: 0,
    uniqueness: 0,
    reportable: false,
    model: null,
    candidates: [],
    best: null,
    alternatives: [],
    ambiguity: { isAmbiguous: false, type: 'none', separationDeg: null },
    credible: null,
    geometry: null,
    conditioning: null,
    fit: null,
    pairs: [],
    pairTausMeasured: [],
    modelSelection: [],
    map: null,
    meta: extra.meta ?? null,
  };
}

/**
 * @param {object} o
 * @param {{pair:number[],tau:number,sigma:number,coherence?:number,peakToFloor?:number}[]} o.observations
 * @param {number[][]} o.positions             mic positions in array frame
 * @param {number} [o.referenceIndex=0]
 * @param {number} o.speedOfSound
 * @param {'far'|'near'|'auto'} [o.model='auto']
 * @param {number} [o.distanceM=2]             near-field start / prior
 * @param {object} [o.grid]
 * @param {object} [o.quality]
 * @param {object} [o.arrayGeometry]           precomputed analyzeArrayGeometry()
 */
export function solveDirection(o) {
  const {
    observations,
    positions,
    referenceIndex = 0,
    speedOfSound,
    model = 'auto',
    distanceM = 2,
    grid: gridOpts = {},
    quality = {},
    arrayGeometry,
  } = o;
  const deltaChi2 = quality.ambiguityDeltaChi2 ?? 9.21;
  const minCoherence = quality.minCoherence ?? 0.1;
  const minPeakToFloor = quality.minPeakToFloor ?? 1.8;
  const maxPairs = quality.maxPairs ?? 64;
  const meta = o.meta ?? null;

  if (!observations || observations.length < 2) {
    if (positions && positions.length === 2) {
      return insufficient(
        'two microphones constrain only one inter-channel TDOA, which leaves a cone of possible directions; no unique bearing can be reported',
        { meta, geometry: analyzeArrayGeometry(positions, quality), ambiguity: { isAmbiguous: true, type: 'cone', separationDeg: null, cone: null } },
      );
    }
    return insufficient(`need at least two usable pair TDOA observations (got ${observations?.length ?? 0})`, { meta });
  }
  if (!speedOfSound || speedOfSound <= 0) throw new Error('speed of sound must be positive');
  if (!positions || positions.length < 2) return insufficient('need at least two microphones', { meta });
  if (positions.length === 2) {
    return insufficient(
      'two microphones constrain only one inter-channel TDOA: this leaves a cone of possible directions, so no unique bearing can be reported',
      { meta, geometry: analyzeArrayGeometry(positions, quality) },
    );
  }

  const geo = arrayGeometry ?? analyzeArrayGeometry(positions, quality);

  const good = observations
    .filter((ob) => Number.isFinite(ob.tau) && ob.sigma > 0
      && (ob.coherence == null || ob.coherence >= minCoherence)
      && (ob.peakToFloor == null || ob.peakToFloor >= minPeakToFloor))
    .sort((a, b) => a.sigma - b.sigma)
    .slice(0, maxPairs);

  if (good.length < 2) {
    return insufficient(
      `only ${good.length} of ${observations.length} pairs pass the quality gate (coherence ≥ ${minCoherence}, peak/floor ≥ ${minPeakToFloor})`,
      { meta },
    );
  }

  const requested = model === 'near' ? ['near'] : model === 'far' ? ['far'] : ['far', 'near'];
  const perModel = [];
  for (const m of requested) {
    const cfg = {
      observations: good,
      positions,
      referenceIndex,
      speedOfSound,
      model: m,
      distanceM,
      grid: gridOpts,
    };
    const g = makeGrid(cfg);
    if (g.azs.length === 0) continue;
    const fits = fitCandidateSet(m, good, cfg, g);
    if (fits.length === 0) continue;
    const nParams = m === 'near' ? 3 : 2;
    perModel.push({ model: m, fits, grid: g, cfg, bic: fits[0].chi2 + nParams * Math.log(good.length), nParams });
  }
  if (perModel.length === 0) return insufficient('all direction fits failed to converge', { meta, geometry: geo });

  perModel.sort((a, b) => a.bic - b.bic);
  let chosen = perModel[0];
  if (model === 'auto') {
    const far = perModel.find((p) => p.model === 'far');
    const near = perModel.find((p) => p.model === 'near');
    // Near-field only wins if it improves the fit enough to justify the extra
    // parameter and its distance estimate is interior to the search range.
    if (near && near.fits[0].nearBoundHit) chosen = far ?? near;
    else if (far && near && near.fits[0].chi2 > far.fits[0].chi2) chosen = far;
  } else if (model === 'near') {
    chosen = perModel.find((p) => p.model === 'near') ?? perModel[0];
  } else if (model === 'far') {
    chosen = perModel.find((p) => p.model === 'far') ?? perModel[0];
  }

  const { fits, grid } = chosen;
  let candidates = fits.slice(0, 6);
  const best0 = candidates[0];

  // Planar arrays have an exact mirror solution across the array plane.
  let mirror = null;
  let mirrorSep = null;
  if (geo.kind === 'planar' || geo.kind === 'near-planar') {
    const d = best0.direction;
    const n = geo.planeNormal;
    const dn = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
    const mirrored = norm([d[0] - 2 * dn * n[0], d[1] - 2 * dn * n[1], d[2] - 2 * dn * n[2]]);
    const angles = unitToAzEl(mirrored);
    mirrorSep = 2 * Math.abs(Math.asin(Math.max(-1, Math.min(1, dn)))) / DEG;
    if (mirrorSep > 2) {
      const params = chosen.model === 'near'
        ? [angles.azDeg, angles.elDeg, best0.params[2]]
        : [angles.azDeg, angles.elDeg];
      try {
        const mf = fitAt(params, chosen.cfg);
        const existing = candidates.find((c) => angularSeparationDeg(c.direction, mf.direction) < 8);
        if (existing) {
          existing.isMirror = true;
        } else {
          mf.isMirror = true;
          mirror = mf;
          candidates.push(mf);
        }
      } catch (e) {
        /* far-field mirror should always fit; ignore numerical failure */
      }
    }
  }

  // Collinear arrays only constrain the angle to the axis: report a cone.
  let cone = null;
  if (geo.kind === 'collinear' || geo.kind === 'near-collinear') {
    const axis = geo.axis;
    const du = Math.abs(best0.direction[0] * axis[0] + best0.direction[1] * axis[1] + best0.direction[2] * axis[2]);
    cone = {
      axis,
      halfAngleDeg: Math.acos(Math.max(-1, Math.min(1, du))) / DEG,
    };
  }

  candidates = dedupeFits(candidates, 8);
  const best = candidates[0];

  // Build credible regions for each candidate basin.
  const credibleList = [];
  for (let i = 0; i < Math.min(3, candidates.length); i++) {
    const ordered = [candidates[i], ...candidates.filter((_, k) => k !== i)];
    const region = profileRegion(ordered, good, chosen.cfg, grid, deltaChi2);
    credibleList.push(region);
  }

  // Rivals are candidates whose chi2 is statistically indistinguishable from best.
  const rivals = candidates.filter((c) => c.chi2 - best.chi2 <= deltaChi2 && angularSeparationDeg(c.direction, best.direction) > 2);
  const bestRegion = credibleList[0];

  // Evidence: how well the measured TDOAs support the model at all.
  const meanCoherence = good.reduce((s, ob) => s + (ob.coherence ?? 1), 0) / good.length;
  const meanPeakToFloor = good.reduce((s, ob) => s + Math.log10(Math.max(1, ob.peakToFloor ?? 3)), 0) / good.length;
  const chi2Red = best.chi2Red;
  let evidence = 1;
  evidence *= Math.exp(-0.5 * Math.max(0, chi2Red - 1) / 2.5);
  evidence *= 0.25 + 0.75 * Math.min(1, meanCoherence / 0.8);
  evidence *= 0.45 + 0.55 * Math.min(1, Math.max(0, (meanPeakToFloor - 0.15) / 0.7));
  evidence *= 0.5 + 0.5 * Math.min(1, good.length / 4);
  const systematic = (quality.systematicTdoaSec ?? 0);
  if (systematic > 0) evidence *= 0.9;
  evidence = Math.max(0, Math.min(1, evidence));

  let uniqueness = 1;
  if (cone) uniqueness = 0;
  else if (rivals.length > 0) uniqueness = Math.min(0.45, 1 / (1 + rivals.length));
  else if (bestRegion && (bestRegion.azExtentDeg > 55 || bestRegion.elExtentDeg > 55)) uniqueness = 0.5;
  else if (geo.kind === 'near-planar' || geo.kind === 'near-collinear') uniqueness *= 0.85;

  let status = 'resolved';
  let reason = '';
  if (cone) {
    status = 'ambiguous';
    reason = `collinear array: direction is only constrained to a cone of half-angle ${cone.halfAngleDeg.toFixed(1)}° around the microphone axis`;
  } else if (rivals.length > 0) {
    status = 'ambiguous';
    const sep = angularSeparationDeg(best.direction, rivals[0].direction);
    reason = `two directions ${sep.toFixed(0)}° apart fit equally well (${geo.kind === 'planar' || geo.kind === 'near-planar' ? 'array-plane mirror' : 'multiple minima'}); new observations are needed to choose`;
  } else if (bestRegion && bestRegion.elExtentDeg > 70) {
    status = 'ambiguous';
    reason = 'elevation is poorly constrained by this array aperture; credible range is broad';
  }
  if (chi2Red > (quality.maxReducedChi2 ?? 12)) {
    status = 'insufficient';
    reason = `measured TDOAs are inconsistent with every direction (chi²/dof = ${chi2Red.toFixed(1)})`;
  }
  const maxSigma = quality.maxSigmaAzDeg ?? 90;
  if (evidence < (quality.minEvidence ?? 0.22) || (Number.isFinite(best.sigmaAzDeg) && best.sigmaAzDeg > maxSigma)) {
    status = 'insufficient';
    if (!reason) reason = 'evidence is too weak for a usable direction estimate';
  }

  const confidence = Math.max(0, Math.min(1, evidence * (0.25 + 0.75 * uniqueness)));
  const reportable = status !== 'insufficient';

  const candidatesOut = candidates.map((c, idx) => ({
    rank: idx + 1,
    azDeg: c.azDeg,
    elDeg: c.elDeg,
    distanceM: c.distanceM,
    direction: c.direction,
    chi2: c.chi2,
    deltaChi2: c.chi2 - best.chi2,
    dof: c.dof,
    chi2Red: c.chi2Red,
    sigmaAzDeg: c.sigmaAzDeg,
    sigmaElDeg: c.sigmaElDeg,
    sigmaDistanceM: c.sigmaDistanceM,
    isMirror: !!c.isMirror,
    credible: credibleList[idx] ?? null,
    // A candidate is *reportable* only when the evidence supports it. Under
    // 'insufficient' these are kept purely for diagnostics (the trial directions the
    // optimiser visited) and must not be rendered or acted on as a bearing.
    plausible: reportable && c.chi2 - best.chi2 <= deltaChi2,
    reportable: reportable && c.chi2 - best.chi2 <= deltaChi2,
  }));

  return {
    status,
    reason,
    confidence,
    evidence,
    uniqueness,
    reportable,
    model: chosen.model,
    nearDistanceM: chosen.model === 'near' ? best.distanceM : null,
    candidates: candidatesOut,
    best: candidatesOut[0],
    alternatives: candidatesOut.slice(1).filter((c) => c.plausible),
    ambiguity: {
      isAmbiguous: status === 'ambiguous',
      type: cone ? 'cone' : rivals.length > 0 ? (geo.kind.includes('planar') ? 'plane-mirror' : 'multi-modal') : 'none',
      separationDeg: rivals[0] ? angularSeparationDeg(best.direction, rivals[0].direction) : null,
      mirrorSeparationDeg: mirrorSep,
      cone,
      deltaChi2,
    },
    credible: bestRegion,
    geometry: {
      kind: geo.kind,
      rank: geo.rank,
      singularValues: geo.singularValues,
      planeNormal: geo.planeNormal,
      axis: geo.axis,
      maxSpanM: geo.maxSpanM,
      outOfPlaneRmsM: geo.outOfPlaneRmsM,
    },
    fit: {
      chi2: best.chi2,
      dof: best.dof,
      chi2Red,
      observationsUsed: good.length,
      observationsOffered: observations.length,
      residualRmsSec: Math.sqrt(best.residuals.reduce((s, r) => s + r * r, 0) / Math.max(1, best.residuals.length)),
    },
    pairs: best.observations,
    pairTausMeasured: good,
    modelSelection: perModel.map((m) => ({ model: m.model, bic: m.bic, chi2: m.fits[0].chi2, distanceM: m.fits[0].distanceM })),
    map: {
      azDeg: grid.azs,
      elDeg: grid.els,
      chi2: grid.points.map((p) => p.chi2),
      bestChi2: best.chi2,
      deltaChi2,
    },
    meta,
  };
}

export { azElToUnit, unitToAzEl, angularSeparationDeg, wrapAz };
