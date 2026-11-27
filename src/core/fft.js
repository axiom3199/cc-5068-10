// Radix-2 complex FFT. In-place, Float64Array, cached twiddles and bit-reversal tables.

const twiddleCache = new Map();
const bitrevCache = new Map();

export function nextPow2(n) {
  let p = 1;
  while (p < n) p <<= 1;
  return p;
}

function getTables(n) {
  let tw = twiddleCache.get(n);
  if (!tw) {
    tw = new Float64Array(n);
    for (let i = 0; i < n / 2; i++) {
      tw[2 * i] = Math.cos((2 * Math.PI * i) / n);
      tw[2 * i + 1] = -Math.sin((2 * Math.PI * i) / n);
    }
    twiddleCache.set(n, tw);
  }
  let br = bitrevCache.get(n);
  if (!br) {
    br = new Uint32Array(n);
    const bits = Math.log2(n);
    for (let i = 0; i < n; i++) {
      let r = 0;
      let v = i;
      for (let b = 0; b < bits; b++) {
        r = (r << 1) | (v & 1);
        v >>= 1;
      }
      br[i] = r;
    }
    bitrevCache.set(n, br);
  }
  return { tw, br };
}

// Forward FFT (sign convention e^{-j...}).
export function fft(re, im) {
  const n = re.length;
  if (n === 1) return;
  if ((n & (n - 1)) !== 0) throw new Error(`FFT length must be a power of two, got ${n}`);
  const { tw, br } = getTables(n);
  for (let i = 0; i < n; i++) {
    const j = br[i];
    if (j > i) {
      let t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (let size = 2; size <= n; size <<= 1) {
    const half = size >> 1;
    const step = n / size;
    for (let i = 0; i < n; i += size) {
      for (let j = 0; j < half; j++) {
        const k = j * step;
        const wr = tw[2 * k];
        const wi = tw[2 * k + 1];
        const a = i + j;
        const b = a + half;
        const xr = re[b] * wr - im[b] * wi;
        const xi = re[b] * wi + im[b] * wr;
        re[b] = re[a] - xr;
        im[b] = im[a] - xi;
        re[a] += xr;
        im[a] += xi;
      }
    }
  }
}

export function ifft(re, im) {
  const n = re.length;
  for (let i = 0; i < n; i++) im[i] = -im[i];
  fft(re, im);
  const inv = 1 / n;
  for (let i = 0; i < n; i++) {
    re[i] *= inv;
    im[i] = -im[i] * inv;
  }
}

export function realFft(signal, nfft) {
  const re = new Float64Array(nfft);
  const im = new Float64Array(nfft);
  re.set(signal.subarray(0, Math.min(signal.length, nfft)));
  fft(re, im);
  return { re, im };
}
