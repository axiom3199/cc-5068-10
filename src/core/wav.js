// Minimal RIFF/WAVE parser for 16/24/32-bit PCM and 32-bit float.
// Returns channel data as Float32Array per channel, normalized to [-1, 1].

export function parseWav(arrayBuffer) {
  const dv = new DataView(arrayBuffer);
  const readTag = (o) => String.fromCharCode(dv.getUint8(o), dv.getUint8(o + 1), dv.getUint8(o + 2), dv.getUint8(o + 3));
  if (readTag(0) !== 'RIFF' || readTag(8) !== 'WAVE') {
    throw new Error('Not a RIFF/WAVE file');
  }
  let offset = 12;
  let fmt = null;
  let dataOffset = -1;
  let dataLength = 0;
  while (offset + 8 <= dv.byteLength) {
    const id = readTag(offset);
    const size = dv.getUint32(offset + 4, true);
    const body = offset + 8;
    if (id === 'fmt ') {
      fmt = {
        audioFormat: dv.getUint16(body, true),
        channels: dv.getUint16(body + 2, true),
        sampleRate: dv.getUint32(body + 4, true),
        byteRate: dv.getUint32(body + 8, true),
        blockAlign: dv.getUint16(body + 12, true),
        bitsPerSample: dv.getUint16(body + 14, true),
      };
    } else if (id === 'data') {
      dataOffset = body;
      dataLength = Math.min(size, dv.byteLength - body);
    }
    offset = body + size + (size & 1);
  }
  if (!fmt || dataOffset < 0) throw new Error('WAVE missing fmt or data chunk');
  if (fmt.channels < 1) throw new Error('WAVE has no channels');

  const bytes = fmt.bitsPerSample >> 3;
  const frames = Math.floor(dataLength / (bytes * fmt.channels));
  const channels = [];
  for (let c = 0; c < fmt.channels; c++) channels.push(new Float32Array(frames));

  const fmtTag = fmt.audioFormat;
  if (fmtTag !== 1 && fmtTag !== 3) throw new Error(`Unsupported WAVE format 0x${fmtTag.toString(16)}`);

  for (let i = 0; i < frames; i++) {
    for (let c = 0; c < fmt.channels; c++) {
      const p = dataOffset + (i * fmt.channels + c) * bytes;
      let v;
      if (fmtTag === 3 && bytes === 4) v = dv.getFloat32(p, true);
      else if (bytes === 2) v = dv.getInt16(p, true) / 32768;
      else if (bytes === 3) {
        let s = dv.getUint8(p) | (dv.getUint8(p + 1) << 8) | (dv.getUint8(p + 2) << 16);
        if (s & 0x800000) s -= 0x1000000;
        v = s / 8388608;
      } else if (bytes === 4) v = dv.getInt32(p, true) / 2147483648;
      else throw new Error(`Unsupported PCM width ${fmt.bitsPerSample}`);
      channels[c][i] = v;
    }
  }
  return { sampleRate: fmt.sampleRate, channels, durationSec: frames / fmt.sampleRate };
}

export function encodeWav(channels, sampleRate) {
  const numCh = channels.length;
  const frames = channels[0].length;
  const buffer = new ArrayBuffer(44 + frames * numCh * 2);
  const dv = new DataView(buffer);
  const tag = (o, s) => { for (let i = 0; i < 4; i++) dv.setUint8(o + i, s.charCodeAt(i)); };
  tag(0, 'RIFF');
  dv.setUint32(4, 36 + frames * numCh * 2, true);
  tag(8, 'WAVE');
  tag(12, 'fmt ');
  dv.setUint32(16, 16, true);
  dv.setUint16(20, 1, true);
  dv.setUint16(22, numCh, true);
  dv.setUint32(24, sampleRate, true);
  dv.setUint32(28, sampleRate * numCh * 2, true);
  dv.setUint16(32, numCh * 2, true);
  dv.setUint16(34, 16, true);
  tag(36, 'data');
  dv.setUint32(40, frames * numCh * 2, true);
  let o = 44;
  for (let i = 0; i < frames; i++) {
    for (let c = 0; c < numCh; c++) {
      const v = Math.max(-1, Math.min(1, channels[c][i]));
      dv.setInt16(o, Math.round(v * 32767), true);
      o += 2;
    }
  }
  return buffer;
}
