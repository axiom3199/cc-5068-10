// Per-channel audition with the Web Audio API. Samples are decoded from the same
// multichannel Float32 buffers the analyser uses, so what you hear is exactly what
// the TDOA estimator saw (the analysis window), not a separate render.
//
// Every channel can be started, stopped, or soloed independently, with a live level
// meter. The graph is rebuilt per playback so sample-rate changes are safe.

export class ChannelPlayer {
  constructor() {
    this.ctx = null;
    this.nodes = new Map();
    this.onLevel = null;
    this._raf = null;
  }

  ensure() {
    if (!this.ctx) this.ctx = new AudioContext();
    if (this.ctx.state === 'suspended') this.ctx.resume();
    return this.ctx;
  }

  /**
   * Play one channel's analysis window, looping, at its true sample rate.
   * @param {number} micIndex
   * @param {Float32Array} samples
   * @param {number} sampleRate
   * @param {string} label
   */
  play(micIndex, samples, sampleRate, label = '') {
    const ctx = this.ensure();
    this.stop(micIndex);
    const buffer = ctx.createBuffer(1, samples.length, sampleRate);
    buffer.copyToChannel(samples, 0);
    const src = ctx.createBufferSource();
    src.buffer = buffer;
    src.loop = true;
    const gain = ctx.createGain();
    gain.gain.value = micIndex === this._solo ? 1 : 0.9;
    const analyser = ctx.createAnalyser();
    analyser.fftSize = 512;
    src.connect(gain).connect(analyser).connect(ctx.destination);
    src.start();
    const node = { src, gain, analyser, label, micIndex };
    this.nodes.set(micIndex, node);
    this._startMeter();
  }

  stop(micIndex) {
    const n = this.nodes.get(micIndex);
    if (!n) return;
    try { n.src.stop(); } catch { /* already stopped */ }
    try { n.src.disconnect(); n.gain.disconnect(); n.analyser.disconnect(); } catch { /* ignore */ }
    this.nodes.delete(micIndex);
    if (this.nodes.size === 0) this._stopMeter();
  }

  stopAll() {
    for (const i of [...this.nodes.keys()]) this.stop(i);
  }

  isPlaying(micIndex) {
    return this.nodes.has(micIndex);
  }

  playingCount() {
    return this.nodes.size;
  }

  /** Loudness in [0,1] for a channel, or 0 if not playing. */
  level(micIndex) {
    const n = this.nodes.get(micIndex);
    if (!n) return 0;
    const buf = new Uint8Array(n.analyser.fftSize);
    n.analyser.getByteTimeDomainData(buf);
    let sum = 0;
    for (const v of buf) { const x = (v - 128) / 128; sum += x * x; }
    return Math.min(1, Math.sqrt(sum / buf.length) * 3.2);
  }

  _startMeter() {
    if (this._raf != null) return;
    const tick = () => {
      if (this.onLevel) {
        const levels = new Map();
        for (const i of this.nodes.keys()) levels.set(i, this.level(i));
        this.onLevel(levels);
      }
      this._raf = this.nodes.size ? requestAnimationFrame(tick) : null;
    };
    this._raf = requestAnimationFrame(tick);
  }

  _stopMeter() {
    if (this._raf != null) { cancelAnimationFrame(this._raf); this._raf = null; }
    if (this.onLevel) this.onLevel(new Map());
  }
}
