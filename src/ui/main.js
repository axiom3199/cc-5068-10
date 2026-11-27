// UI controller.
//
// Truth lives ONLY here, in the world frame, for synthesising the recording and for
// the verification overlays. The inference boundary is explicit: analyzeDirection()
// receives a truth-free object {recording, geometry, options}. Nothing carrying
// source coordinates crosses into it.
//
//   world truth + array layout + camera pose
//        -> renderScene (array-frame recording; the only caller of the generator)
//        -> optional physical channel rewire
//        -> analyzeDirection({recording, geometry, options})     [no truth]
//        -> DirectionTracker (world hypotheses; hold/slew policy)
//        -> CameraController + VirtualCamera (rate-limited slew)
//        -> canvases

import { renderScene } from '../core/synthesize.js';
import { analyzeDirection } from '../core/analysis.js';
import { DirectionTracker } from '../core/tracker.js';
import { DirectionJumpDiagnostics } from '../core/jumpdiag.js';
import { VirtualCamera, CameraController } from '../core/camera.js';
import { analyzeArrayGeometry } from '../core/solver.js';
import { planProbeForDecision } from '../core/disambiguation.js';
import { azElToUnit, unitToAzEl, arrayToWorld, worldAzElToArray } from '../core/geometry.js';
import { drawPlan, drawBearing, drawXcorr, drawCamera } from './render.js';
import { ChannelPlayer } from './audio.js';

const FS = 48000;
const C_DEFAULT = 343;
const RECORDING_SEC = 1.2;
const NEUTRAL_NEAR_PRIOR_M = 1.5; // fixed prior; never the true distance

const LAYOUTS = {
  tetra: {
    label: '3D 四面体',
    positions: [[0.05, 0.04, 0.03], [0.05, -0.03, -0.04], [-0.05, 0.03, -0.04], [-0.05, -0.04, 0.03]],
  },
  square: {
    label: '水平方阵',
    positions: [[-0.06, 0, -0.06], [0.06, 0, -0.06], [0.06, 0, 0.06], [-0.06, 0, 0.06]],
  },
  triangle: {
    label: '三角阵',
    positions: [[0, 0.03, 0.08], [-0.07, -0.02, -0.04], [0.07, -0.02, -0.04]],
  },
  line: {
    label: '共线阵',
    positions: [[-0.09, 0, 0], [-0.03, 0, 0], [0.03, 0, 0], [0.09, 0, 0]],
  },
  stereo: {
    label: '双话筒',
    positions: [[-0.07, 0, 0], [0.07, 0, 0]],
  },
};

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 1) => (v == null || !Number.isFinite(v) ? '—' : v.toFixed(d));
const panDisplay = (a) => ((((a + 180) % 360) + 360) % 360) - 180;

const state = {
  layout: 'tetra',
  positions: LAYOUTS.tetra.positions.map((p) => p.slice()),
  // world-frame source (render + verification only)
  source: { azDeg: 55, elDeg: 18, distanceM: null, snrDb: 24, present: true },
  camera: { panDeg: 0, tiltDeg: 0 },
  window: { startSec: 0, lengthSec: 0.12 },
  c: C_DEFAULT,
  wiring: null, // wiring[mic] = recording channel physically carrying that mic; null = identity
  follow: false,
  scene: null,
  recording: null,
  channelMap: null,
  worldTruth: null,
  result: null,
  analysis: null,
  tracker: null,
  decision: null,
  virtualCamera: null,
  controller: null,
  selectedMic: null,
  selectedPairIdx: 0,
  probe: null,
  history: [],
  lastAnalysisPose: null,
  reanalysisPending: false,
  lastError: null,
  jumpDiag: null,        // DirectionJumpDiagnostics, lazily created
  pendingNote: {},       // operator-change note consumed by the next observe()
  selectedJumpEvent: null,
  fault: null,           // one-shot injected fault, consumed by the next runAnalysis
};

const player = new ChannelPlayer();

// ------------------------------------------------------------------ helpers
function arrayGeometry() {
  return analyzeArrayGeometry(state.positions);
}

function channelMap() {
  const n = state.positions.length;
  return state.wiring ? state.wiring.slice() : state.positions.map((_, i) => i);
}

function clampWindow() {
  const maxStart = Math.max(0, RECORDING_SEC - state.window.lengthSec - 0.01);
  if (state.window.startSec > maxStart) state.window.startSec = maxStart;
  const el = $('winStart');
  if (el) el.value = String(state.window.startSec);
  const out = $('winStartOut');
  if (out) out.textContent = `${state.window.startSec.toFixed(2)} s`;
}

// ---------------------------------------------------------------- synthesize
function rebuildScene() {
  const t = state.source;
  // World source -> array frame, because the array is rigidly mounted to the camera.
  const local = worldAzElToArray(t.azDeg, t.elDeg, state.camera.panDeg, state.camera.tiltDeg);
  const sc = renderScene({
    positions: state.positions,
    source: {
      azDeg: local.azDeg,
      elDeg: local.elDeg,
      distanceM: t.distanceM,
      nearField: t.distanceM != null,
    },
    sampleRate: FS,
    speedOfSound: state.c,
    durationSec: RECORDING_SEC,
    snrDb: t.present ? t.snrDb : -40,
    seed: 11,
    bandLow: 300,
    bandHigh: 6000,
    equalPower: t.distanceM == null,
  });
  if (!t.present) {
    for (const ch of sc.channels) for (let i = 0; i < ch.length; i++) ch[i] *= 0.02;
  }
  state.scene = sc;
  state.worldTruth = { azDeg: t.azDeg, elDeg: t.elDeg };

  // Physical wiring: place mic m's samples into recording channel wiring[m]. The
  // channel map (metadata) is what lets the analyser undo the rewire.
  const map = channelMap();
  const n = state.positions.length;
  const recChannels = new Array(n);
  for (let mic = 0; mic < n; mic++) recChannels[map[mic]] = sc.channels[mic];
  state.recording = { channels: recChannels, sampleRate: FS };
  state.channelMap = map;
  return sc;
}

// ------------------------------------------------------------------ analysis
// One-shot physical fault injection: rewire or silence recording channels
// WITHOUT updating the declared channel map, so the analyser sees wrong data.
function applyFault() {
  const f = state.fault;
  if (!f) return;
  state.fault = null;
  const chans = state.recording.channels;
  if (f.type === 'swap') {
    const [a, b] = f.mics;
    const ca = state.channelMap[a];
    const cb = state.channelMap[b];
    [chans[ca], chans[cb]] = [chans[cb], chans[ca]];
  } else if (f.type === 'drop') {
    const ch = state.channelMap[f.mic];
    chans[ch] = new Float32Array(chans[ch].length);
  }
}

function runAnalysis() {
  state.lastError = null;
  try {
    clampWindow();
    const sc = rebuildScene();
    applyFault();
    const startSample = Math.round(state.window.startSec * FS);
    const frameLength = Math.round(state.window.lengthSec * FS);

    // Inference input: recording + geometry + analysis options. No source truth.
    const analysis = analyzeDirection({
      recording: {
        channels: state.recording.channels,
        sampleRate: FS,
        startSample,
      },
      geometry: {
        positions: state.positions,
        channelMap: state.channelMap,
        speedOfSound: state.c,
        referenceIndex: 0,
      },
      options: {
        frameLength,
        model: 'auto',
        bandLow: 300,
        bandHigh: 6000,
        nearDistanceM: NEUTRAL_NEAR_PRIOR_M,
      },
    });
    state.analysis = analysis;
    state.result = analysis.direction;

    if (!state.tracker) state.tracker = new DirectionTracker();
    const decision = state.tracker.update(
      analysis.direction,
      { panDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg },
      performance.now() / 1000,
    );
    state.decision = decision;

    // Direction-jump diagnostics: watch solver/truth divergence against the
    // deferred-attribution reference. The pending note marks operator changes
    // (geometry/wiring/source) so expected jumps are not false-reported.
    if (!state.jumpDiag) state.jumpDiag = new DirectionJumpDiagnostics();
    const jumpEvent = state.jumpDiag.observe({
      analysis,
      pose: { panDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg },
      decision,
      recording: { channels: state.recording.channels, sampleRate: FS },
      note: state.pendingNote,
    });
    state.pendingNote = {};
    if (jumpEvent) state.selectedJumpEvent = jumpEvent.id;

    state.history.unshift({
      t: new Date().toLocaleTimeString('zh-CN', { hour12: false }),
      status: analysis.direction.status,
      action: decision.action,
      reason: decision.reason,
    });
    state.history = state.history.slice(0, 14);

    if (state.follow && !state.probe) applyFollow(decision);

    state.lastAnalysisPose = { panDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg };
    renderAll();
    window.__doaLastResult = { result: state.result, decision, analysis };
    return analysis;
  } catch (e) {
    state.lastError = e;
    renderError(e);
    throw e;
  }
}

function applyFollow(decision) {
  if (!state.virtualCamera) {
    state.virtualCamera = new VirtualCamera({}, { yawDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg });
    state.controller = new CameraController(state.virtualCamera);
  }
  // Keep the virtual camera's pose in sync with anything the user changed.
  state.virtualCamera.yawDeg = state.camera.panDeg;
  state.virtualCamera.tiltDeg = state.camera.tiltDeg;
  state.controller.command(decision);
}

// ---------------------------------------------------------------- rendering
function renderAll() {
  renderPlan();
  renderBearing();
  renderXcorr();
  renderCameraView();
  renderResults();
  renderChannels();
  renderPairTable();
  renderJumpDiag();
  renderDiagnostics();
  renderDecision();
  renderStatus();
}

function statusLabel(r) {
  return { resolved: '已定位', ambiguous: '存在歧义', insufficient: '证据不足' }[r.status] ?? r.status;
}

function renderStatus() {
  const r = state.result;
  const pill = $('statusPill');
  if (!r) { pill.className = 'pill pill-idle'; pill.textContent = '待分析'; return; }
  const cls = { resolved: 'pill-resolved', ambiguous: 'pill-ambiguous', insufficient: 'pill-insufficient' }[r.status] ?? 'pill-idle';
  let label = statusLabel(r);
  if (state.decision?.action === 'hold' && r.status !== 'insufficient') label += ' · 暂缓转向';
  if (state.decision?.action === 'slew') label += ' · 转向中';
  pill.className = `pill ${cls}`;
  pill.textContent = label;
  $('poseReadout').textContent = `云台 pan ${fmt(panDisplay(state.camera.panDeg))}° / tilt ${fmt(state.camera.tiltDeg)}°`;
}

function renderPlan() {
  const canvas = $('planCanvas');
  const info = drawPlan(canvas, {
    positions: state.positions,
    result: state.result,
    truth: state.source.present ? state.worldTruth : null,
    pose: { panDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg },
    selectedMic: state.selectedMic,
  });
  canvas._planInfo = info;
  const geo = arrayGeometry();
  $('geometryNote').textContent =
    `阵列类型：${kindLabel(geo.kind)}（${state.positions.length} 通道，最大基线 ${(geo.maxSpanM * 100).toFixed(1)} cm，离面 ${(geo.outOfPlaneRmsM * 100).toFixed(1)} cm）`;
}

function kindLabel(kind) {
  return {
    '3d': '三维阵列（可全向解算）',
    planar: '平面阵列（存在镜像解）',
    'near-planar': '近平面阵列（近似镜像）',
    collinear: '共线阵列（锥形歧义）',
    'near-collinear': '近共线阵列（近锥形）',
    none: '无话筒',
  }[kind] ?? kind;
}

function renderBearing() {
  drawBearing($('bearingCanvas'), {
    result: state.result,
    truth: state.source.present ? state.worldTruth : null,
    pose: { panDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg },
    tracks: state.tracker?.tracks,
  });
}

function renderXcorr() {
  const pairs = state.analysis?.tdoa?.pairs ?? [];
  const sel = $('pairSelect');
  const key = pairs.map((p) => p.pair.join('-')).join(',');
  if (sel._key !== key) {
    sel._key = key;
    sel.innerHTML = '';
    pairs.forEach((p, i) => {
      const opt = document.createElement('option');
      opt.value = String(i);
      opt.textContent = p.pair.map((m) => `M${m}`).join('–');
      sel.appendChild(opt);
    });
    state.selectedPairIdx = 0;
    sel.value = '0';
  }
  const idx = Math.min(state.selectedPairIdx, Math.max(0, pairs.length - 1));
  let pair = pairs[idx] ?? null;
  if (pair) {
    pair = {
      ...pair,
      sampleRate: state.analysis.metadata.sampleRate,
      allowedTdoaBandSec: allowedBandForPair(pair.pair),
    };
  }
  drawXcorr($('xcorrCanvas'), pair);
  if (pair) {
    const obs = state.result?.pairs?.find((x) => x.pair[0] === pair.pair[0] && x.pair[1] === pair.pair[1]);
    $('xcorrNote').textContent =
      `实测峰 ${(pair.lagSec * 1e6).toFixed(2)} µs · 相干 ${pair.coherence.toFixed(2)} · 峰/底 ${pair.peakToFloor.toFixed(1)}`
      + (obs ? ` · 与拟合方向残差 ${(obs.residual * 1e6).toFixed(2)} µs` : '')
      + (pair.allowedTdoaBandSec ? ' · 蓝带 = 拟合方向允许时差（±3 样本）' : '');
  } else {
    $('xcorrNote').textContent = '暂无通道对数据';
  }
}

function allowedBandForPair([a, b]) {
  const obs = state.result?.pairs?.find((x) => x.pair[0] === a && x.pair[1] === b);
  if (!obs || !Number.isFinite(obs.tauPred)) return null;
  const ds = 3 / state.analysis.metadata.sampleRate;
  return [obs.tauPred - ds, obs.tauPred + ds];
}

function candidatesWorld() {
  if (!state.result?.candidates) return [];
  return state.result.candidates
    .filter((c) => c.plausible)
    .map((c) => {
      const w = arrayToWorld(azElToUnit(c.azDeg, c.elDeg), state.camera.panDeg, state.camera.tiltDeg);
      return { ...c, ...unitToAzEl(w) };
    });
}

function renderCameraView() {
  drawCamera($('cameraCanvas'), {
    pose: { panDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg },
    truth: state.source.present ? state.worldTruth : null,
    result: { decision: state.decision },
    candidatesWorld: candidatesWorld(),
  });
}

function renderResults() {
  const box = $('resultSummary');
  const list = $('candidateList');
  const detail = $('resultDetail');
  const r = state.result;
  if (!r) {
    box.className = 'summary summary-idle';
    box.innerHTML = '<div class="summary-status">尚未分析</div><div class="summary-reason">点击「分析当前窗口」。结果只来自录音通道与几何参数。</div>';
    list.innerHTML = '';
    detail.innerHTML = '';
    return;
  }
  const label = {
    resolved: '方向已定位',
    ambiguous: '方向存在歧义（保留候选）',
    insufficient: '证据不足，拒绝给出唯一方向',
  }[r.status] ?? r.status;
  box.className = `summary ${r.status}`;
  box.innerHTML = `
    <div class="summary-status">${label}
      <span class="badge">${r.model === 'near' ? '近场' : r.model === 'far' ? '远场' : '—'}</span>
      <span class="badge">置信 ${fmt(r.confidence * 100, 0)}%</span></div>
    <div class="summary-reason">${escapeHtml(r.reason || '拟合良好；候选与可信范围见下。')}</div>`;

  list.innerHTML = '';
  if (r.status === 'insufficient') {
    const note = document.createElement('div');
    note.className = 'cand';
    note.style.gridTemplateColumns = '1fr';
    note.innerHTML = '<span class="angles">没有足够可靠的声源证据；下方候选仅为诊断残差，未采信为方向。</span>';
    list.appendChild(note);
  }
  for (const c of r.candidates) {
    if (!c.plausible && r.status !== 'insufficient') continue;
    const div = document.createElement('div');
    div.className = `cand ${c.rank === 1 && c.plausible ? 'rank1' : ''}`;
    const cred = c.credible
      ? `可信 az ${c.credible.azRangeDeg.map((v) => fmt(v, 0)).join('…')}° · el ${c.credible.elRangeDeg.map((v) => fmt(v, 0)).join('…')}°`
      : '无可信区间';
    div.innerHTML = `
      <span class="rank">#${c.rank}</span>
      <span class="angles">方位 ${fmt(c.azDeg)}° · 仰角 ${fmt(c.elDeg)}°
        ${c.isMirror ? '<span class="mirror-tag">（镜像解）</span>' : ''}
        ${!c.plausible ? '<span class="mirror-tag">（未采信）</span>' : ''}
        <br><small>${cred}</small></span>
      <span class="meta">χ² ${fmt(c.chi2, 2)}<br>Δχ² ${fmt(c.deltaChi2, 2)}<br>σ ${fmt(c.sigmaAzDeg)}°</span>`;
    list.appendChild(div);
  }

  if (!r.fit) { detail.innerHTML = ''; return; }
  const rows = [
    ['拟合 χ²/dof', `${fmt(r.fit.chi2, 2)} / ${r.fit.dof} = ${fmt(r.fit.chi2Red, 2)}`],
    ['残差 RMS', `${(r.fit.residualRmsSec * 1e6).toFixed(2)} µs`],
    ['使用通道对', `${r.fit.observationsUsed} / ${r.fit.observationsOffered}`],
    ['阵列几何', kindLabel(r.geometry?.kind)],
    ['证据强度', `${fmt(r.evidence * 100, 0)}%`],
    ['方向唯一性', `${fmt(r.uniqueness * 100, 0)}%`],
  ];
  if (r.nearDistanceM != null) rows.push(['估计距离', `${fmt(r.nearDistanceM, 2)} m`]);
  if (r.ambiguity?.cone) rows.push(['锥半角', `${fmt(r.ambiguity.cone.halfAngleDeg)}°`]);
  if (r.ambiguity?.separationDeg != null) rows.push(['候选间隔', `${fmt(r.ambiguity.separationDeg, 0)}°`]);
  detail.innerHTML = rows.map(([k, v]) => `<span>${escapeHtml(k)}</span><b>${escapeHtml(v)}</b>`).join('');
}

function renderChannels() {
  const container = $('channelList');
  if (!state.recording) return;
  const map = state.channelMap;
  container.innerHTML = '';
  state.positions.forEach((pos, mic) => {
    const recCh = map[mic];
    const row = document.createElement('div');
    row.className = `ch ${player.isPlaying(mic) ? 'playing' : ''}`;
    const pathCm = nearPathCm(pos);
    row.innerHTML = `
      <span class="idx">M${mic}</span>
      <span class="name">话筒 ${mic} <small>← 录制 ch${recCh}</small>
        ${pathCm != null ? `<small>· 声程 ${pathCm.toFixed(1)} cm</small>` : ''}
      </span>
      <span class="level"><i data-mic="${mic}"></i></span>
      <button class="btn play" type="button" data-mic="${mic}">${player.isPlaying(mic) ? '停止' : '试听'}</button>`;
    container.appendChild(row);
  });
  container.querySelectorAll('button.play').forEach((btn) => {
    btn.addEventListener('click', () => {
      const mic = Number(btn.dataset.mic);
      if (player.isPlaying(mic)) player.stop(mic);
      else {
        const recCh = state.channelMap[mic];
        const start = Math.round(state.window.startSec * FS);
        const len = Math.round(state.window.lengthSec * FS);
        const src = state.recording.channels[recCh];
        const buf = src.slice(start, Math.min(src.length, start + len));
        player.play(mic, buf, FS, `M${mic}`);
      }
      renderChannels();
    });
  });
}

function nearPathCm(pos) {
  if (state.source.distanceM == null) return null;
  const local = worldAzElToArray(state.source.azDeg, state.source.elDeg, state.camera.panDeg, state.camera.tiltDeg);
  const d = azElToUnit(local.azDeg, local.elDeg);
  const r = state.source.distanceM;
  return Math.hypot(r * d[0] - pos[0], r * d[1] - pos[1], r * d[2] - pos[2]) * 100;
}

function renderPairTable() {
  const container = $('pairTable');
  if (!state.result?.pairs?.length) {
    container.innerHTML = '<div class="pt-row head"><span>通道对</span><span>暂无通过质量门限的通道对</span></div>';
    return;
  }
  const head = '<div class="pt-row head"><span>通道对</span><span>实测 µs</span><span>拟合 µs</span><span>残差 µs</span><span>权重</span></div>';
  const rows = state.result.pairs.map((p) => {
    const bad = Math.abs(p.residual) > 3 * p.sigma;
    return `<div class="pt-row ${bad ? 'bad' : ''}"><span>M${p.pair[0]}–M${p.pair[1]}</span>
      <span>${(p.tauObs * 1e6).toFixed(2)}</span>
      <span>${(p.tauPred * 1e6).toFixed(2)}</span>
      <span>${(p.residual * 1e6).toFixed(2)}</span>
      <span>σ${(p.sigma * 1e6).toFixed(2)}</span></div>`;
  });
  container.innerHTML = head + rows.join('');
}

function renderDiagnostics() {
  const a = state.analysis;
  const box = $('diagBox');
  if (!a) { box.textContent = '等待分析…'; return; }
  const m = a.metadata;
  const lines = [
    `采样率      ${m.sampleRate} Hz`,
    `窗口        ${(m.startSample / m.sampleRate).toFixed(3)} s + ${(m.frameLength / m.sampleRate).toFixed(3)} s（${m.frameLength} 样本）`,
    `声速        ${m.speedOfSound} m/s`,
    `通道映射    channelMap[mic] = [${m.channelMap.join(', ')}]`,
    `参考通道    M${m.referenceIndex}`,
    `频带/加权   ${m.bandLow}–${m.bandHigh} Hz · ${String(m.weighting).toUpperCase()}`,
    `模型        ${a.direction.model ?? '—'}`,
    `候选数      ${a.direction.candidates.length}`,
    `通道对      ${a.tdoa.pairs.length}`,
    `状态        ${a.direction.status}`,
  ];
  if (a.direction.reason) lines.push(`判定        ${a.direction.reason}`);
  box.textContent = lines.join('\n');
}

// --------------------------------------------------------- jump diagnostics
const JUMP_CAUSE_LABEL = {
  'channel-swap': '通道互换',
  'mic-dropout': '话筒掉线',
  'sign-error': '时差符号错误',
  'strong-reflection': '强反射',
  'multi-peak-ambiguity': '多峰歧义',
  'track-switch': '轨迹切换',
  unexplained: '未明原因',
  'axis-transform': '阵列/布局变换（预期）',
  'source-moved': '声源移动（预期）',
  'source-gone': '声源消失（预期）',
};
const JUMP_STAGE_LABEL = { tdoa: '时差估计', solve: '方向解算', track: '跟踪决策' };

function renderJumpDiag() {
  const diag = state.jumpDiag;
  const budgetEl = $('jumpBudget');
  const alertEl = $('jumpAlert');
  const listEl = $('jumpEventList');
  const detailEl = $('jumpEventDetail');
  if (!budgetEl) return;
  if (!diag) {
    budgetEl.textContent = '';
    alertEl.className = 'jump-alert idle';
    alertEl.textContent = '等待分析…';
    listEl.innerHTML = '';
    detailEl.innerHTML = '';
    return;
  }
  const stats = diag.stats();
  budgetEl.textContent =
    `帧 ${stats.frames} · 事件 ${stats.events}/${stats.maxEvents}（异常 ${stats.anomalies} / 预期 ${stats.expected}）`
    + ` · 留存片段 ${stats.segmentsHeld}/${stats.maxSegments}`
    + (stats.droppedEvents || stats.droppedSegments
      ? ` · 已按预算丢弃 事件${stats.droppedEvents} 片段${stats.droppedSegments}` : '');

  const events = diag.events();
  const latestAnomaly = events.find((e) => e.kind === 'anomaly');
  if (latestAnomaly) {
    alertEl.className = 'jump-alert anomaly';
    alertEl.textContent =
      `异常 #${latestAnomaly.id}：${JUMP_CAUSE_LABEL[latestAnomaly.cause] ?? latestAnomaly.cause}`
      + `（阶段：${JUMP_STAGE_LABEL[latestAnomaly.stage] ?? latestAnomaly.stage}`
      + (latestAnomaly.jumpDeg != null ? `，跳变 ${latestAnomaly.jumpDeg}°` : '')
      + (latestAnomaly.implicatedMics.length ? `，涉及 ${latestAnomaly.implicatedMics.map((m) => `M${m}`).join('/')}）` : '）');
  } else {
    alertEl.className = 'jump-alert ok';
    alertEl.textContent = events.length
      ? '仅有预期内变化（布局调整 / 声源移动），无异常跳变。'
      : '未检测到方向跳变。';
  }

  listEl.innerHTML = '';
  for (const ev of events) {
    const div = document.createElement('div');
    div.className = `jump-item ${ev.kind} ${state.selectedJumpEvent === ev.id ? 'selected' : ''}`;
    const mics = ev.implicatedMics.length ? ` · ${ev.implicatedMics.map((m) => `M${m}`).join('/')}` : '';
    div.innerHTML = `<span class="j-id">#${ev.id}</span>
      <span class="j-cause">${JUMP_CAUSE_LABEL[ev.cause] ?? ev.cause}${ev.count > 1 ? ` ×${ev.count}` : ''}</span>
      <span class="j-meta">${JUMP_STAGE_LABEL[ev.stage] ?? ev.stage}${ev.jumpDeg != null ? ` · ${ev.jumpDeg}°` : ''}${mics}</span>`;
    div.addEventListener('click', () => {
      state.selectedJumpEvent = state.selectedJumpEvent === ev.id ? null : ev.id;
      renderJumpDiag();
    });
    listEl.appendChild(div);
  }
  if (!events.length) listEl.innerHTML = '<div class="jump-empty">暂无事件</div>';

  const sel = events.find((e) => e.id === state.selectedJumpEvent);
  if (!sel) { detailEl.innerHTML = ''; return; }
  const rows = [];
  rows.push(`<div class="jd-row"><span>结论</span><b>${escapeHtml(sel.reason)}</b></div>`);
  if (sel.referenceAzEl) rows.push(`<div class="jd-row"><span>参照来向（世界）</span><b>方位 ${fmt(sel.referenceAzEl.azDeg)}° · 仰角 ${fmt(sel.referenceAzEl.elDeg)}°</b></div>`);
  if (sel.reportedAzEl) rows.push(`<div class="jd-row"><span>报告来向（世界）</span><b>方位 ${fmt(sel.reportedAzEl.azDeg)}° · 仰角 ${fmt(sel.reportedAzEl.elDeg)}°</b></div>`);
  if (sel.tauConsistency) {
    const bad = sel.tauConsistency.perPair.filter((p) => !p.consistent);
    rows.push(`<div class="jd-row"><span>时差一致性</span><b>${Math.round(sel.tauConsistency.consistentFrac * 100)}% 通道对与参照一致`
      + (bad.length ? `；偏离：${bad.map((p) => `M${p.pair[0]}–M${p.pair[1]}(z=${p.z})`).join('，')}` : '')
      + `</b></div>`);
  }
  if (sel.branchSupport?.reference) {
    rows.push(`<div class="jd-row"><span>分支支撑</span><b>参照 ${Math.round(sel.branchSupport.reference.frac * 100)}%`
      + (sel.branchSupport.reported ? ` · 报告 ${Math.round(sel.branchSupport.reported.frac * 100)}%` : '')
      + (sel.branchSupport.retainable ? ' · 参照分支仍可保留' : ' · 参照分支失去麦对支撑')
      + `</b></div>`);
  }
  const peakKeys = Object.keys(sel.peaks ?? {});
  if (peakKeys.length) {
    rows.push(`<div class="jd-row"><span>候选峰</span><b>${peakKeys.map((k) => {
      const [a, b] = k.split('-');
      return `M${a}–M${b}: ${sel.peaks[k].map((p) => `${(p.lagSec * 1e6).toFixed(0)}µs×${p.prominence}`).join(' / ')}`;
    }).join('；')}</b></div>`);
  }
  const v = sel.verifyResult;
  if (v) {
    rows.push(`<div class="jd-row"><span>复跑核验</span><b class="${v.conclusion === 'confirmed' ? 'ok-text' : 'warn-text'}">${
      { confirmed: '已确认', refuted: '被证伪', 'not-applicable': '不适用' }[v.conclusion] ?? v.conclusion
    }：${v.steps.map((s) => `${s.step}${s.ok == null ? '' : s.ok ? '✓' : '✗'}`).join(' → ')}</b></div>`);
  }
  const canReplay = !!sel.segment;
  detailEl.innerHTML = rows.join('')
    + `<div class="jd-actions">
        <button class="btn btn-ghost" type="button" data-act="locate">定位到窗口</button>
        <button class="btn btn-ghost" type="button" data-act="replay" ${canReplay ? '' : 'disabled'}>${canReplay ? '回放该段' : (sel.segmentEvicted ? '片段已按预算丢弃' : '无留存片段')}</button>
        <button class="btn btn-ghost" type="button" data-act="verify">复跑核验</button>
      </div>`;
  detailEl.querySelectorAll('button').forEach((btn) => {
    btn.addEventListener('click', () => jumpEventAction(sel, btn.dataset.act));
  });
}

function jumpEventAction(ev, act) {
  const diag = state.jumpDiag;
  if (!diag) return;
  if (act === 'locate') {
    const ctx = ev.solveContext;
    state.window.startSec = Math.min(
      Math.max(0, RECORDING_SEC - state.window.lengthSec - 0.01),
      ctx.startSample / ctx.sampleRate,
    );
    state.window.lengthSec = ctx.frameLength / ctx.sampleRate;
    $('winStart').value = String(state.window.startSec);
    $('winStartOut').textContent = `${state.window.startSec.toFixed(2)} s`;
    $('winLen').value = String(state.window.lengthSec);
    $('winLenOut').textContent = `${state.window.lengthSec.toFixed(3)} s`;
    renderJumpDiag();
  } else if (act === 'replay' && ev.segment) {
    // Mix the retained mic-ordered segment down to mono and play it back.
    const n = ev.segment.frameLength;
    const mix = new Float32Array(n);
    for (const ch of ev.segment.channels) {
      for (let i = 0; i < Math.min(n, ch.length); i++) mix[i] += ch[i] / ev.segment.channels.length;
    }
    player.play('jump', mix, ev.segment.sampleRate, `事件 #${ev.id} 片段`);
  } else if (act === 'verify') {
    diag.verify(ev.id);
    renderJumpDiag();
  }
}

function renderDecision() {
  const box = $('decisionBox');
  const log = $('decisionLog');
  if (!state.decision) { box.textContent = '尚未运行分析。'; log.innerHTML = ''; return; }
  const d = state.decision;
  const act = d.action === 'slew' ? '转向' : '暂缓';
  box.innerHTML = `<span class="act ${d.action}">${act}</span> · ${escapeHtml(d.reason)}`;
  log.innerHTML = state.history.map((h) =>
    `<li><span class="t">${h.t}</span><span>${h.status}</span><span>${h.action === 'slew' ? '转向' : '暂缓'} · ${escapeHtml(h.reason)}</span></li>`).join('');
}

function renderError(e) {
  const box = $('resultSummary');
  box.className = 'summary insufficient';
  box.innerHTML = `<div class="summary-status">分析失败</div><div class="summary-reason">${escapeHtml(e.message)}</div>`;
  $('statusPill').className = 'pill pill-insufficient';
  $('statusPill').textContent = '分析失败';
  $('diagBox').textContent = String(e.stack || e.message || e);
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

// -------------------------------------------------------------------- follow
let lastFrame = performance.now();
let reanalysisTimer = null;

function scheduleAnalysis(ms = 160) {
  if (state.reanalysisPending) return;
  state.reanalysisPending = true;
  clearTimeout(reanalysisTimer);
  reanalysisTimer = setTimeout(() => {
    state.reanalysisPending = false;
    try { runAnalysis(); } catch { /* error already rendered */ }
  }, ms);
}

function poseDelta(a, b) {
  if (!a || !b) return 999;
  const dp = Math.abs((((a.panDeg - b.panDeg) % 360) + 540) % 360 - 180);
  return Math.hypot(dp, a.tiltDeg - b.tiltDeg);
}

function followLoop() {
  const now = performance.now();
  const dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;

  if (state.probe) {
    stepProbe(dt);
  } else if (state.follow && state.controller) {
    const res = state.controller.step(dt);
    if (res.movedDeg > 0.005) {
      state.camera.panDeg = state.virtualCamera.yawDeg;
      state.camera.tiltDeg = state.virtualCamera.tiltDeg;
      updateCameraControls();
      $('poseReadout').textContent = `云台 pan ${fmt(panDisplay(state.camera.panDeg))}° / tilt ${fmt(state.camera.tiltDeg)}°`;
      renderPlan();
      renderBearing();
      renderCameraView();
      renderStatus();
    }
    if (res.settled && poseDelta(state.lastAnalysisPose, state.camera) > 2) {
      // Camera settled at a new pose: re-observe so the core sees the new geometry
      // and the cross-pose consistency check gains pose spread.
      scheduleAnalysis(220);
    }
  }
  requestAnimationFrame(followLoop);
}

// --------------------------------------------------------------------- probe
function startProbe() {
  if (!state.result?.candidates?.length) {
    $('followNote').textContent = '当前没有可用候选，无法规划探针。';
    return;
  }
  const pose = { panDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg };
  const decision = state.decision ?? {
    ambiguity: { isAmbiguous: true, candidates: state.result.candidates.filter((c) => c.plausible) },
  };
  const plan = planProbeForDecision({
    decision,
    pose,
    planeNormalArray: arrayGeometry().planeNormal ?? [0, 1, 0],
    probeDeg: 16,
  });
  if (!plan) {
    $('followNote').textContent = '当前观测不是可分离的镜像歧义（或预计位移太小），无需通过转头排除。';
    return;
  }
  state.follow = false;
  $('followBtn').textContent = '跟拍：关';
  $('followBtn').classList.remove('btn-primary');
  state.probe = {
    target: plan.newPose,
    axis: plan.axis,
    deltaDeg: plan.deltaDeg,
    predictedSpurShiftDeg: plan.predictedSpurShiftDeg,
    started: performance.now(),
  };
  $('followNote').textContent =
    `探针：${plan.axis === 'pan' ? '偏航' : '俯仰'} ${plan.deltaDeg > 0 ? '+' : ''}${plan.deltaDeg}°，预计把镜像支路推开 ${plan.predictedSpurShiftDeg.toFixed(1)}°。`;
}

function stepProbe(dt) {
  const p = state.probe;
  if (!p) return;
  if (!state.virtualCamera) {
    state.virtualCamera = new VirtualCamera({}, { yawDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg });
    state.controller = new CameraController(state.virtualCamera);
  }
  state.virtualCamera.yawDeg = state.camera.panDeg;
  state.virtualCamera.tiltDeg = state.camera.tiltDeg;
  const res = state.virtualCamera.step({ yawDeg: p.target.yawDeg, tiltDeg: p.target.tiltDeg }, dt);
  state.camera.panDeg = state.virtualCamera.yawDeg;
  state.camera.tiltDeg = state.virtualCamera.tiltDeg;
  updateCameraControls();
  renderPlan();
  renderBearing();
  renderCameraView();
  renderStatus();

  const dYaw = Math.abs((((p.target.yawDeg - state.camera.panDeg) % 360) + 540) % 360 - 180);
  const dTilt = Math.abs(p.target.tiltDeg - state.camera.tiltDeg);
  if ((res.settled && dYaw < 1.5 && dTilt < 1.5) || (performance.now() - p.started) > 4000) {
    const elapsed = ((performance.now() - p.started) / 1000).toFixed(2);
    state.probe = null;
    runAnalysis();
    $('followNote').textContent =
      `探针完成（${elapsed}s），已在新姿态重新分析；若一致性判据锁定，相机将转向真源、镜像支路被排除。`;
  }
}

// ------------------------------------------------------------------ controls
function updateCameraControls() {
  $('camPan').value = String(Math.round(panDisplay(state.camera.panDeg) * 2) / 2);
  $('camTilt').value = String(Math.round(state.camera.tiltDeg * 2) / 2);
  $('camPanOut').textContent = `${fmt(panDisplay(state.camera.panDeg))}°`;
  $('camTiltOut').textContent = `${fmt(state.camera.tiltDeg)}°`;
}

function syncSourceControls() {
  const s = state.source;
  $('srcAz').value = String(s.azDeg);
  $('srcEl').value = String(s.elDeg);
  $('srcDist').value = s.distanceM == null ? '0' : String(s.distanceM);
  $('srcSnr').value = String(s.snrDb);
  $('srcOn').checked = s.present;
  $('srcAzOut').textContent = `${fmt(s.azDeg)}°`;
  $('srcElOut').textContent = `${fmt(s.elDeg)}°`;
  $('srcDistOut').textContent = s.distanceM == null ? '远场' : `${fmt(s.distanceM, 2)} m`;
  $('srcSnrOut').textContent = `${s.snrDb} dB`;
}

function readSourceControls() {
  const s = state.source;
  s.azDeg = Number($('srcAz').value);
  s.elDeg = Number($('srcEl').value);
  const dist = Number($('srcDist').value);
  s.distanceM = dist === 0 ? null : dist;
  s.snrDb = Number($('srcSnr').value);
  s.present = $('srcOn').checked;
  syncSourceControls();
}

function updateChannelMapNote(verbose = true) {
  const el = $('channelMapNote');
  if (!state.wiring) {
    el.textContent = verbose
      ? '录制通道与话筒编号一致（恒等映射 channelMap[i] = i）。'
      : '通道已恢复为恒等映射。';
    return;
  }
  const pairs = state.channelMap.map((recCh, mic) => `ch${recCh}=M${mic}`);
  el.textContent = `录制通道已被物理重排：${pairs.join('，')}。分析内核据此映射还原，结果应保持正确。`;
}

function buildMicEditor() {
  const container = $('micEditor');
  container.innerHTML = '';
  state.positions.forEach((p, i) => {
    const wrap = document.createElement('div');
    wrap.className = 'mic-edit';
    wrap.innerHTML = `<b>M${i}</b> x<input data-i="${i}" data-k="0" value="${(p[0] * 100).toFixed(1)}" />`
      + ` y<input data-i="${i}" data-k="1" value="${(p[1] * 100).toFixed(1)}" />`
      + ` z<input data-i="${i}" data-k="2" value="${(p[2] * 100).toFixed(1)}" />cm`;
    container.appendChild(wrap);
  });
  container.querySelectorAll('input').forEach((inp) => {
    inp.addEventListener('change', () => {
      const i = Number(inp.dataset.i);
      const k = Number(inp.dataset.k);
      const v = Number(inp.value);
      if (!Number.isFinite(v)) return;
      state.positions[i][k] = v / 100;
      state.tracker = null;
      state.jumpDiag = null;
      state.probe = null;
      state.pendingNote = { ...state.pendingNote, geometryChanged: true };
      runAnalysis();
    });
  });
}

function setupMicDrag() {
  const plan = $('planCanvas');
  let dragging = -1;
  const hit = (e) => {
    const info = plan._planInfo;
    if (!info) return -1;
    let best = -1;
    let bd = Infinity;
    state.positions.forEach((p, i) => {
      const [sx, sy] = info.toScreen(p[0], p[2]);
      const d = Math.hypot(sx - e.offsetX, sy - e.offsetY);
      if (d < bd) { bd = d; best = i; }
    });
    return bd < 18 ? best : -1;
  };
  plan.addEventListener('pointerdown', (e) => {
    const i = hit(e);
    if (i < 0) return;
    dragging = i;
    state.selectedMic = i;
    try { plan.setPointerCapture(e.pointerId); } catch { /* ignore */ }
    renderPlan();
  });
  plan.addEventListener('pointermove', (e) => {
    if (dragging < 0) return;
    const info = plan._planInfo;
    if (!info) return;
    const x = (e.offsetX - info.cx) / info.scale;
    const z = (info.cy - e.offsetY) / info.scale;
    state.positions[dragging][0] = Math.round(x * 1000) / 1000;
    state.positions[dragging][2] = Math.round(z * 1000) / 1000;
    renderPlan();
  });
  const end = () => {
    if (dragging < 0) return;
    dragging = -1;
    state.tracker = null;
    state.jumpDiag = null;
    state.probe = null;
    state.pendingNote = { ...state.pendingNote, geometryChanged: true };
    buildMicEditor();
    try { runAnalysis(); } catch { /* rendered */ }
  };
  plan.addEventListener('pointerup', end);
  plan.addEventListener('pointercancel', end);
}

function bindControls() {
  document.querySelectorAll('.btn-chip[data-layout]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const name = btn.dataset.layout;
      if (!LAYOUTS[name]) return;
      state.layout = name;
      state.positions = LAYOUTS[name].positions.map((p) => p.slice());
      state.tracker = null;
      state.jumpDiag = null;
      state.probe = null;
      state.pendingNote = { ...state.pendingNote, geometryChanged: true };
      document.querySelectorAll('.btn-chip[data-layout]').forEach((b) => b.classList.toggle('active', b === btn));
      buildMicEditor();
      runAnalysis();
    });
  });

  ['srcAz', 'srcEl', 'srcDist', 'srcSnr'].forEach((id) =>
    $(id).addEventListener('input', () => {
      readSourceControls();
      state.pendingNote = { ...state.pendingNote, sourceChanged: true };
    }));
  $('srcOn').addEventListener('change', () => {
    readSourceControls();
    state.pendingNote = { ...state.pendingNote, sourceChanged: true };
    state.tracker = null;
    state.jumpDiag = null;
    state.probe = null;
    runAnalysis();
  });

  $('camPan').addEventListener('input', () => {
    state.camera.panDeg = Number($('camPan').value);
    updateCameraControls();
    state.probe = null;
    scheduleAnalysis(140);
  });
  $('camTilt').addEventListener('input', () => {
    state.camera.tiltDeg = Number($('camTilt').value);
    updateCameraControls();
    state.probe = null;
    scheduleAnalysis(140);
  });

  $('followBtn').addEventListener('click', () => {
    state.follow = !state.follow;
    $('followBtn').textContent = `跟拍：${state.follow ? '开' : '关'}`;
    $('followBtn').classList.toggle('btn-primary', state.follow);
    if (state.follow) {
      $('followNote').textContent = '跟拍已开启：每帧结果驱动跟踪器决定转向或暂缓；歧义时保持上次可信方向，不追单个候选。';
      if (!state.virtualCamera) {
        state.virtualCamera = new VirtualCamera({}, { yawDeg: state.camera.panDeg, tiltDeg: state.camera.tiltDeg });
        state.controller = new CameraController(state.virtualCamera);
      }
      runAnalysis();
    } else {
      $('followNote').textContent = '跟拍已关闭：云台保持当前位置。';
    }
  });

  $('probeBtn').addEventListener('click', startProbe);
  $('recenterBtn').addEventListener('click', () => {
    state.probe = null;
    state.camera.panDeg = 0;
    state.camera.tiltDeg = 0;
    state.follow = false;
    $('followBtn').textContent = '跟拍：关';
    $('followBtn').classList.remove('btn-primary');
    if (state.virtualCamera) { state.virtualCamera.yawDeg = 0; state.virtualCamera.tiltDeg = 0; }
    if (state.controller) state.controller.commandedWorld = null;
    updateCameraControls();
    runAnalysis();
  });

  $('winStart').addEventListener('input', () => {
    state.window.startSec = Number($('winStart').value);
    $('winStartOut').textContent = `${state.window.startSec.toFixed(2)} s`;
  });
  $('winLen').addEventListener('input', () => {
    state.window.lengthSec = Number($('winLen').value);
    $('winLenOut').textContent = `${state.window.lengthSec.toFixed(3)} s`;
  });
  $('cInput').addEventListener('input', () => {
    state.c = Number($('cInput').value);
    $('cOut').textContent = `${state.c} m/s`;
  });

  $('analyzeBtn').addEventListener('click', () => runAnalysis());

  $('scrambleBtn').addEventListener('click', () => {
    const n = state.positions.length;
    const perm = [...Array(n).keys()];
    for (let i = n - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [perm[i], perm[j]] = [perm[j], perm[i]];
    }
    // wiring[mic] = recording channel that physically carries mic mic
    state.wiring = perm;
    state.pendingNote = { ...state.pendingNote, wiringChanged: true };
    updateChannelMapNote();
    runAnalysis();
  });
  $('resetChBtn').addEventListener('click', () => {
    state.wiring = null;
    state.pendingNote = { ...state.pendingNote, wiringChanged: true };
    updateChannelMapNote(false);
    runAnalysis();
  });

  $('pairSelect').addEventListener('change', (e) => {
    state.selectedPairIdx = Number(e.target.value);
    renderXcorr();
  });

  // Fault injection: physically corrupt the next analysis frame (one shot) so
  // the jump diagnostics can be exercised from the UI.
  $('faultSwapBtn')?.addEventListener('click', () => {
    const n = state.positions.length;
    if (n < 2) return;
    const a = Math.floor(Math.random() * n);
    let b = Math.floor(Math.random() * (n - 1));
    if (b >= a) b += 1;
    state.fault = { type: 'swap', mics: [a, b] };
    runAnalysis();
  });
  $('faultDropBtn')?.addEventListener('click', () => {
    const n = state.positions.length;
    if (n < 2) return;
    state.fault = { type: 'drop', mic: Math.floor(Math.random() * n) };
    runAnalysis();
  });

  $('stopBtn').addEventListener('click', () => {
    player.stopAll();
    renderChannels();
  });

  $('helpBtn').addEventListener('click', () => $('helpDialog').showModal());

  setupMicDrag();

  window.addEventListener('resize', debounce(() => {
    renderPlan();
    renderBearing();
    renderXcorr();
    renderCameraView();
  }, 120));

  syncSourceControls();
  updateCameraControls();
  updateChannelMapNote();
}

function debounce(fn, ms) {
  let timer = null;
  return (...args) => {
    clearTimeout(timer);
    timer = setTimeout(() => fn(...args), ms);
  };
}

// ----------------------------------------------------------------- bootstrap
player.onLevel = (levels) => {
  for (const [mic, lvl] of levels) {
    const el = document.querySelector(`.level i[data-mic="${mic}"]`);
    if (el) el.style.width = `${Math.round(lvl * 100)}%`;
  }
};

function init() {
  document.querySelectorAll('.btn-chip[data-layout]').forEach((b) => {
    b.classList.toggle('active', b.dataset.layout === state.layout);
  });
  bindControls();
  buildMicEditor();
  try { runAnalysis(); } catch (e) { console.error(e); }
  requestAnimationFrame(followLoop);
}

init();

// Documented test surface for the headless UI smoke test.
window.__doa = {
  state,
  runAnalysis,
  setScene({ layout, positions, source, camera, c, window: win, wiring } = {}) {
    const note = {};
    if (layout && LAYOUTS[layout]) {
      state.layout = layout;
      state.positions = LAYOUTS[layout].positions.map((p) => p.slice());
      note.geometryChanged = true;
    }
    if (positions) {
      state.positions = positions.map((p) => p.slice());
      note.geometryChanged = true;
    }
    if (source) {
      Object.assign(state.source, source);
      note.sourceChanged = true;
    }
    if (camera) Object.assign(state.camera, camera);
    if (c != null) state.c = c;
    if (win) Object.assign(state.window, win);
    if (wiring !== undefined) {
      state.wiring = wiring;
      note.wiringChanged = true;
    }
    state.tracker = null;
    state.jumpDiag = null;
    state.probe = null;
    state.pendingNote = { ...state.pendingNote, ...note };
    syncSourceControls();
    $('cInput').value = String(state.c);
    $('cOut').textContent = `${state.c} m/s`;
    updateCameraControls();
    buildMicEditor();
    updateChannelMapNote();
    return runAnalysis();
  },
  probe: startProbe,
  candidatesWorld,
  setFollow(on) {
    if (!!on !== state.follow) $('followBtn').click();
  },
  // Direction-jump diagnostics surface.
  jumpEvents() {
    return state.jumpDiag ? state.jumpDiag.events() : [];
  },
  jumpStats() {
    return state.jumpDiag ? state.jumpDiag.stats() : null;
  },
  verifyJump(id) {
    return state.jumpDiag ? state.jumpDiag.verify(id) : null;
  },
  selectJumpEvent(id) {
    state.selectedJumpEvent = id;
    renderJumpDiag();
  },
  injectFault(f) {
    state.fault = f;
    return runAnalysis();
  },
};
