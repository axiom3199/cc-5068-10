// Canvas renderers for the four views. Pure drawing functions: they receive plain
// data and never touch the inference modules. World directions are unit vectors in
// { x: right, y: up, z: forward }.

const COL = {
  grid: '#1a2230',
  grid2: '#222c3c',
  axis: '#334055',
  text: '#8b98a9',
  textDim: '#5b6675',
  accent: '#4cc2ff',
  accentSoft: 'rgba(76,194,255,0.16)',
  cand: '#d292ff',
  candSoft: 'rgba(210,146,255,0.14)',
  truth: '#e3b341',
  cam: '#58a6ff',
  usable: '#3fb950',
  warn: '#d29922',
  bad: '#f85149',
  mic: '#7ee787',
  micDim: '#3a4a3f',
};

function dpr(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const w = canvas.clientWidth || canvas.width;
  const h = (w * canvas.height) / canvas.width;
  if (canvas.width !== Math.round(w * ratio)) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  return { ratio, w, h };
}

function setup(canvas) {
  const { ratio, w, h } = dpr(canvas);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  const style = getComputedStyle(canvas);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}

const DEG = Math.PI / 180;

// -------------------------------------------------------------- plan view
/**
 * Top-down array layout. Mics are draggable when opts.onDragMic is provided.
 * @param {HTMLCanvasElement} canvas
 * @param {object} o
 * @param {number[][]} o.positions
 * @param {object} [o.result]        direction result (array frame)
 * @param {object|null} o.truth      world az/el (drawn only as a verification overlay)
 * @param {{panDeg:number,tiltDeg:number}} o.pose
 * @param {number} [o.selectedMic]
 * @param {(i:number,pos:number[])=>void} [o.onDragMic]
 */
export function drawPlan(canvas, o) {
  const { ctx, w, h } = setup(canvas);
  const pad = 46;
  const cx = w / 2;
  const cy = h / 2;
  // fit all mic positions in view; keep a pleasant scale
  let maxR = 0.12;
  for (const p of o.positions) maxR = Math.max(maxR, Math.hypot(p[0], p[2]) + 0.02);
  const scale = Math.min((w - pad * 2) / (2 * maxR), (h - pad * 2) / (2 * maxR));
  const toScreen = (x, z) => [cx + x * scale, cy - z * scale];

  // grid rings
  ctx.strokeStyle = COL.grid;
  ctx.lineWidth = 1;
  for (const r of [0.05, 0.1, 0.2, 0.4]) {
    if (r * scale > Math.min(w, h) / 2) continue;
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
  }
  // axes: +z forward (up), +x right
  ctx.strokeStyle = COL.grid2;
  ctx.beginPath();
  ctx.moveTo(pad, cy); ctx.lineTo(w - pad, cy);
  ctx.moveTo(cx, h - pad); ctx.lineTo(cx, pad);
  ctx.stroke();
  ctx.fillStyle = COL.textDim;
  ctx.font = '10px ui-monospace, monospace';
  ctx.fillText('+x', w - pad + 2, cy + 3);
  ctx.fillText('+z', cx + 4, pad - 6);
  ctx.fillText('−x', 6, cy + 3);
  ctx.fillText('−z', cx + 4, h - pad + 14);

  // array centroid marker
  ctx.fillStyle = COL.textDim;
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillText('阵列中心', cx + 8, cy - 6);

  // direction candidate sectors (array frame): az 0 is +z, az +90 is +x.
  const drawAz = (azDeg, color, width, dash = [], alpha = 1) => {
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    const a = azDeg * DEG;
    const x = Math.sin(a);
    const z = Math.cos(a);
    const R = Math.min(w, h) / 2 - 14;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + x * R, cy - z * R);
    ctx.stroke();
    ctx.restore();
  };

  // credible arc for the best candidate
  if (o.result?.credible) {
    const [a0, a1] = o.result.credible.azRangeDeg;
    ctx.fillStyle = COL.accentSoft;
    const R = Math.min(w, h) / 2 - 14;
    const start = (a0 - 90) * DEG;
    const end = (a1 - 90) * DEG;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, R, start, end, false);
    ctx.closePath();
    ctx.fill();
  }

  // all candidates, then highlight alternatives
  if (o.result?.candidates) {
    for (const c of o.result.candidates.slice().reverse()) {
      const isBest = c.rank === 1;
      if (!c.plausible) continue;
      drawAz(c.azDeg, isBest ? COL.accent : COL.cand, isBest ? 2.2 : 1.4, c.isMirror ? [6, 5] : [], isBest ? 1 : 0.9);
      const a = c.azDeg * DEG;
      const R = Math.min(w, h) / 2 - 22;
      const lx = cx + Math.sin(a) * R;
      const ly = cy - Math.cos(a) * R;
      ctx.fillStyle = isBest ? COL.accent : COL.cand;
      ctx.font = '10px ui-monospace, monospace';
      ctx.fillText(isBest ? '首选' : (c.isMirror ? '镜像' : `候选${c.rank}`), lx + 4, ly);
    }
  }

  // world-truth reference (verification overlay): rotate world az/el into the array
  // frame so it lines up with what the inference shows.
  if (o.truth) {
    const d = azElToUnit(o.truth.azDeg, o.truth.elDeg);
    const R = rotMat(o.pose.panDeg, o.pose.tiltDeg);
    const local = [
      R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
      R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
      R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
    ];
    const az = Math.atan2(local[0], local[2]) / DEG;
    drawAz(az, COL.truth, 1.2, [2, 4], 0.85);
    ctx.fillStyle = COL.truth;
    ctx.font = '10px ui-monospace, monospace';
    const R2 = Math.min(w, h) / 2 - 40;
    ctx.fillText('真值(校验)', cx + Math.sin(az * DEG) * R2 + 4, cy - Math.cos(az * DEG) * R2);
  }

  // camera forward in array frame (camera looks along -pan of its own frame)
  {
    const fwd = [0, 0, 1]; // array frame the camera boresight at pose 0 is +z
    void fwd;
    // Draw the camera's current boresight expressed in the array frame.
    const R = rotMat(o.pose.panDeg, o.pose.tiltDeg);
    const wf = [Math.sin(o.pose.panDeg * DEG) * Math.cos(o.pose.tiltDeg * DEG), Math.sin(o.pose.tiltDeg * DEG), Math.cos(o.pose.panDeg * DEG) * Math.cos(o.pose.tiltDeg * DEG)];
    const local = [
      R[0][0] * wf[0] + R[1][0] * wf[1] + R[2][0] * wf[2],
      R[0][1] * wf[0] + R[1][1] * wf[1] + R[2][1] * wf[2],
      R[0][2] * wf[0] + R[1][2] * wf[1] + R[2][2] * wf[2],
    ];
    drawAz(Math.atan2(local[0], local[2]) / DEG, COL.cam, 1.4, [3, 3], 0.9);
  }

  // microphones
  o.positions.forEach((p, i) => {
    const [sx, sy] = toScreen(p[0], p[2]);
    const sel = o.selectedMic === i;
    ctx.beginPath();
    ctx.arc(sx, sy, sel ? 8 : 6, 0, Math.PI * 2);
    ctx.fillStyle = sel ? COL.accent : '#0b0f14';
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = COL.mic;
    ctx.stroke();
    ctx.fillStyle = sel ? '#04121a' : COL.mic;
    ctx.font = '600 9px ui-monospace, monospace';
    ctx.fillText(`M${i}`, sx - 6, sy + 3);
    if (p[1] !== 0) {
      ctx.fillStyle = COL.textDim;
      ctx.font = '9px ui-monospace, monospace';
      ctx.fillText(`y${(p[1] * 100).toFixed(1)}`, sx + 9, sy - 6);
    }
  });

  // return screen transform for drag hit-testing
  return { toScreen, scale, cx, cy };
}

// ----------------------------------------------------------- bearing view
/**
 * Two-panel direction view: horizontal (az) on top, elevation profile below.
 * @param {HTMLCanvasElement} canvas
 * @param {object} o
 * @param {object|null} o.result
 * @param {object|null} o.truth      world az/el, converted to array frame for overlay
 * @param {{panDeg:number,tiltDeg:number}} o.pose
 * @param {object[]} [o.tracks]
 */
export function drawBearing(canvas, o) {
  const { ctx, w, h } = setup(canvas);
  const topH = h * 0.62;
  const cx = w / 2;
  const cy = topH / 2 + 8;
  const radius = Math.min(w / 2 - 26, topH / 2 - 22);

  // background polar grid
  ctx.strokeStyle = COL.grid;
  ctx.lineWidth = 1;
  for (const r of [0.25, 0.5, 0.75, 1]) {
    ctx.beginPath();
    ctx.arc(cx, cy, radius * r, 0, Math.PI * 2);
    ctx.stroke();
  }
  for (let a = 0; a < 360; a += 30) {
    const rad = (a - 90) * DEG;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(rad) * radius, cy + Math.sin(rad) * radius);
    ctx.stroke();
    ctx.fillStyle = COL.textDim;
    ctx.font = '9px ui-monospace, monospace';
    ctx.fillText(`${a}°`, cx + Math.cos(rad) * (radius + 4) - 8, cy + Math.sin(rad) * (radius + 4) + 3);
  }
  ctx.fillStyle = COL.textDim;
  ctx.font = '10px ui-monospace, monospace';
  ctx.fillText('前 +z', cx - 12, cy - radius - 6);
  ctx.fillText('右 +x', cx + radius + 6, cy + 3);

  const R = rotMat(o.pose.panDeg, o.pose.tiltDeg);
  const toArray = (az, el) => {
    const d = azElToUnit(az, el);
    return [
      R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
      R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
      R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2],
    ];
  };
  const plot = (d, color, width, dash = [], alpha = 1, dotR = 3) => {
    const az = Math.atan2(d[0], d[2]);
    const r = Math.sqrt(d[0] * d[0] + d[2] * d[2]);
    const x = cx + Math.sin(az) * radius * (0.35 + 0.65 * r);
    const y = cy - Math.cos(az) * radius * (0.35 + 0.65 * r);
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(x, y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y, dotR, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
    return { x, y };
  };

  // credible wedge for the best candidate
  if (o.result?.best && o.result.credible) {
    const [a0, a1] = o.result.credible.azRangeDeg;
    const start = (a0 - 90) * DEG;
    const end = (a1 - 90) * DEG;
    ctx.fillStyle = COL.accentSoft;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, radius, start, end, false);
    ctx.closePath();
    ctx.fill();
  }

  if (o.result?.candidates) {
    for (const c of o.result.candidates.slice().reverse()) {
      if (!c.plausible) continue;
      const d = azElToUnit(c.azDeg, c.elDeg);
      const best = c.rank === 1;
      const p = plot(d, best ? COL.accent : COL.cand, best ? 2.2 : 1.3, c.isMirror ? [5, 4] : [], best ? 1 : 0.9);
      ctx.fillStyle = best ? COL.accent : COL.cand;
      ctx.font = '10px ui-monospace, monospace';
      ctx.fillText(best ? `首选 ${c.azDeg.toFixed(0)},${c.elDeg.toFixed(0)}` : `#${c.rank}`, p.x + 5, p.y - 3);
    }
  }
  if (o.truth) {
    plot(toArray(o.truth.azDeg, o.truth.elDeg), COL.truth, 1.2, [2, 4], 0.85, 2.5);
  }
  // Camera boresight expressed in the ARRAY frame. The array is rigidly mounted to
  // the camera, so its boresight is always +z here, whichever way the rig is turned.
  plot([0, 0, 1], COL.cam, 1.2, [3, 3], 0.8, 2);

  // tracks (world-frame hypotheses) projected into the array frame
  if (o.tracks?.length) {
    for (const t of o.tracks) {
      const wd = azElToUnit(t.azDeg, t.elDeg);
      const local = [
        R[0][0] * wd[0] + R[1][0] * wd[1] + R[2][0] * wd[2],
        R[0][1] * wd[0] + R[1][1] * wd[1] + R[2][1] * wd[2],
        R[0][2] * wd[0] + R[1][2] * wd[1] + R[2][2] * wd[2],
      ];
      plot(normalize(local), COL.usable, 1, [1, 4], 0.55, 2);
    }
  }

  // ---------------------------------------------------- elevation profile
  const baseY = topH + 22;
  const profH = h - baseY - 22;
  const left = 34;
  const right = w - 12;
  ctx.strokeStyle = COL.grid;
  ctx.beginPath();
  ctx.moveTo(left, baseY + profH / 2);
  ctx.lineTo(right, baseY + profH / 2);
  ctx.stroke();
  ctx.fillStyle = COL.textDim;
  ctx.font = '9px ui-monospace, monospace';
  for (const el of [-60, -30, 0, 30, 60]) {
    const y = baseY + profH / 2 - (el / 90) * (profH / 2);
    ctx.fillText(`${el}°`, 4, y + 3);
    ctx.strokeStyle = el === 0 ? COL.grid2 : COL.grid;
    ctx.beginPath();
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
    ctx.stroke();
  }
  ctx.fillStyle = COL.textDim;
  ctx.fillText('仰角剖面', left, baseY - 6);
  const azToX = (az) => left + ((az + 180) / 360) * (right - left);
  const elToY = (el) => baseY + profH / 2 - (el / 90) * (profH / 2);
  const markProfile = (az, el, color, dash = [], label) => {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.setLineDash(dash);
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.moveTo(azToX(az), baseY);
    ctx.lineTo(azToX(az), baseY + profH);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(azToX(az), elToY(el), 3, 0, Math.PI * 2);
    ctx.fill();
    if (label) {
      ctx.font = '9px ui-monospace, monospace';
      ctx.fillText(label, azToX(az) + 4, elToY(el) - 5);
    }
    ctx.restore();
  };
  if (o.result?.candidates) {
    for (const c of o.result.candidates) {
      if (!c.plausible) continue;
      markProfile(c.azDeg, c.elDeg, c.rank === 1 ? COL.accent : COL.cand, c.isMirror ? [5, 4] : [], `#${c.rank}`);
    }
  }
  if (o.truth) {
    const t = toArray(o.truth.azDeg, o.truth.elDeg);
    const az = Math.atan2(t[0], t[2]) / DEG;
    const el = Math.asin(Math.max(-1, Math.min(1, t[1]))) / DEG;
    markProfile(az, el, COL.truth, [2, 4], '真值');
  }
}

// ------------------------------------------------------------ cross-correlation
export function drawXcorr(canvas, pair) {
  const { ctx, w, h } = setup(canvas);
  const left = 44;
  const right = w - 12;
  const top = 12;
  const bot = h - 26;
  if (!pair) {
    ctx.fillStyle = COL.textDim;
    ctx.font = '12px ui-monospace, monospace';
    ctx.fillText('选择一对通道，或先做分析', left, h / 2);
    return;
  }
  const values = pair.values;
  let lo = Infinity;
  let hi = -Infinity;
  for (const v of values) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  const span = hi - lo || 1;
  const xFor = (lagSample) => left + ((lagSample + pair.maxLag) / (2 * pair.maxLag)) * (right - left);
  const yFor = (v) => bot - ((v - lo) / span) * (bot - top);

  // zero-lag line
  ctx.strokeStyle = COL.grid2;
  ctx.beginPath();
  ctx.moveTo(xFor(0), top);
  ctx.lineTo(xFor(0), bot);
  ctx.stroke();

  // geometry-allowed lag band (from predicted pair TDOA if provided)
  if (pair.allowedTdoaBandSec) {
    const [t0, t1] = pair.allowedTdoaBandSec;
    const s0 = xFor(t0 * pair.sampleRate);
    const s1 = xFor(t1 * pair.sampleRate);
    ctx.fillStyle = 'rgba(88,166,255,0.10)';
    ctx.fillRect(Math.min(s0, s1), top, Math.abs(s1 - s0), bot - top);
  }

  // curve
  ctx.strokeStyle = COL.accent;
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  pair.lags.forEach((lag, i) => {
    const x = xFor(lag);
    const y = yFor(values[i]);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // measured peak
  const px = xFor(pair.lagSamples);
  const py = yFor(pair.peakValue);
  ctx.strokeStyle = COL.usable;
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.moveTo(px, top);
  ctx.lineTo(px, bot);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = COL.usable;
  ctx.beginPath();
  ctx.arc(px, py, 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.font = '10px ui-monospace, monospace';
  ctx.fillText(`${pair.pair.map((i) => `M${i}`).join('–')}  ${(pair.lagSec * 1e6).toFixed(1)} µs`, px + 6, py - 8);

  // axes labels
  ctx.fillStyle = COL.textDim;
  ctx.font = '9px ui-monospace, monospace';
  ctx.fillText(`−${(pair.maxLag / pair.sampleRate * 1e6).toFixed(0)}µs`, left - 6, h - 10);
  ctx.fillText('0', xFor(0) - 2, h - 10);
  ctx.fillText(`+${(pair.maxLag / pair.sampleRate * 1e6).toFixed(0)}µs`, right - 40, h - 10);
  ctx.fillText('GCC-PHAT 归一化相关', left, h - 10);
}

// ------------------------------------------------------------- camera view
export function drawCamera(canvas, o) {
  const { ctx, w, h } = setup(canvas);
  const { pose, fovDeg = { h: 55, v: 34 }, truth, result, candidatesWorld } = o;
  const cx = w / 2;
  const cy = h / 2;
  ctx.strokeStyle = COL.grid2;
  ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
  // grid
  ctx.strokeStyle = COL.grid;
  for (let i = 1; i < 4; i++) {
    ctx.beginPath();
    ctx.moveTo((w / 4) * i, 0);
    ctx.lineTo((w / 4) * i, h);
    ctx.moveTo(0, (h / 4) * i);
    ctx.lineTo(w, (h / 4) * i);
    ctx.stroke();
  }
  ctx.fillStyle = COL.textDim;
  ctx.font = '9px ui-monospace, monospace';
  ctx.fillText(`pan ${pose.panDeg.toFixed(1)}°  tilt ${pose.tiltDeg.toFixed(1)}°`, 8, 14);
  ctx.fillText(`HFOV ${fovDeg.h}°  VFOV ${fovDeg.v}°`, 8, h - 8);
  // center reticle
  ctx.strokeStyle = COL.cam;
  ctx.beginPath();
  ctx.moveTo(cx - 10, cy); ctx.lineTo(cx - 3, cy);
  ctx.moveTo(cx + 3, cy); ctx.lineTo(cx + 10, cy);
  ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy - 3);
  ctx.moveTo(cx, cy + 3); ctx.lineTo(cx, cy + 10);
  ctx.stroke();

  const project = (azDeg, elDeg) => {
    // world az/el into camera frame using the pose
    const dWorld = azElToUnit(azDeg, elDeg);
    const R = rotMat(pose.panDeg, pose.tiltDeg);
    const dCam = [
      R[0][0] * dWorld[0] + R[1][0] * dWorld[1] + R[2][0] * dWorld[2],
      R[0][1] * dWorld[0] + R[1][1] * dWorld[1] + R[2][1] * dWorld[2],
      R[0][2] * dWorld[0] + R[1][2] * dWorld[1] + R[2][2] * dWorld[2],
    ];
    const az = Math.atan2(dCam[0], dCam[2]) / DEG;
    const el = Math.asin(Math.max(-1, Math.min(1, dCam[1]))) / DEG;
    const u = az / (fovDeg.h / 2);
    const v = el / (fovDeg.v / 2);
    return { x: cx + u * (w / 2), y: cy - v * (h / 2), in: Math.abs(u) <= 1 && Math.abs(v) <= 1, az, el };
  };

  // draw truth as a "locked object" box
  if (truth) {
    const p = project(truth.azDeg, truth.elDeg);
    if (p.in) {
      const bw = 44;
      const bh = 34;
      ctx.strokeStyle = COL.truth;
      ctx.lineWidth = 1.4;
      ctx.strokeRect(p.x - bw / 2, p.y - bh / 2, bw, bh);
      ctx.fillStyle = COL.truth;
      ctx.font = '9px ui-monospace, monospace';
      ctx.fillText('锁住对象(真值)', p.x - bw / 2, p.y - bh / 2 - 4);
    } else {
      ctx.fillStyle = COL.truth;
      ctx.font = '10px ui-monospace, monospace';
      const dir = p.az > 0 ? '→ 右' : '← 左';
      const text = p.el > 0 ? '↑ 上' : '↓ 下';
      ctx.fillText(`锁住对象在画面外 ${dir}${text} (${p.az.toFixed(0)}°, ${p.el.toFixed(0)}°)`, 12, h - 26);
    }
  }

  // candidates (world) as colored boxes
  const cands = candidatesWorld ?? [];
  for (const c of cands) {
    const p = project(c.azDeg, c.elDeg);
    if (!p.in) continue;
    ctx.strokeStyle = c.rank === 1 ? COL.accent : COL.cand;
    ctx.setLineDash(c.isMirror ? [4, 4] : []);
    ctx.lineWidth = 1.2;
    ctx.strokeRect(p.x - 22, p.y - 18, 44, 36);
    ctx.setLineDash([]);
    ctx.fillStyle = c.rank === 1 ? COL.accent : COL.cand;
    ctx.font = '9px ui-monospace, monospace';
    ctx.fillText(c.rank === 1 ? '首选' : (c.isMirror ? '镜像' : `候选${c.rank}`), p.x - 20, p.y - 22);
  }

  // Decision overlay
  if (result?.decision) {
    ctx.font = '10px ui-monospace, monospace';
    ctx.fillStyle = result.decision.action === 'slew' ? COL.usable : COL.cam;
    ctx.fillText(`决策：${result.decision.action === 'slew' ? '转向' : '暂缓'} - ${result.decision.reason}`, 8, 28);
  }
}

// -------------------------------------------------------------- helpers
function rotMat(panDeg, tiltDeg) {
  const y = panDeg * DEG;
  const t = tiltDeg * DEG;
  const cy = Math.cos(y), sy = Math.sin(y);
  const ct = Math.cos(t), st = Math.sin(t);
  return [
    [cy, -sy * st, sy * ct],
    [0, ct, st],
    [-sy, -cy * st, cy * ct],
  ];
}

function azElToUnit(azDeg, elDeg) {
  const az = azDeg * DEG;
  const el = elDeg * DEG;
  return [Math.sin(az) * Math.cos(el), Math.sin(el), Math.cos(az) * Math.cos(el)];
}

function normalize(v) {
  const n = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / n, v[1] / n, v[2] / n];
}

export const palette = COL;
