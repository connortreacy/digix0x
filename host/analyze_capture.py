#!/usr/bin/env python3
"""Analyse a UMC1820 capture of the board's DAC output.
Usage: analyze_capture.py capture.wav [channel]
Reports discontinuity (click) counts vs thresholds, held-sample runs,
HF band energy and quiet-floor level. Reference: a clean host render
has max sample-to-sample jump ~2000 (int16) and no jumps >12000."""
import sys
import numpy as np
from scipy.io import wavfile

def main():
    fs, d = wavfile.read(sys.argv[1])
    ch = int(sys.argv[2]) if len(sys.argv) > 2 else None
    x = d.astype(np.float64)
    if x.ndim > 1:
        rms_ch = [float(np.sqrt((c**2).mean())) for c in x.T]
        ch = int(np.argmax(rms_ch)) if ch is None else ch
        x = x[:, ch]
    d1 = np.abs(np.diff(x))
    win = fs // 20
    wr = np.array([np.sqrt((x[i:i+win]**2).mean()) for i in range(0, len(x)-win, win)])
    print(f"{sys.argv[1]} ch={ch} fs={fs} dur={len(x)/fs:.1f}s rms={np.sqrt((x**2).mean()):.0f} "
          f"peak={np.abs(x).max():.0f} quietest={wr.min():.0f}")
    for th in (4000, 8000, 12000, 16000):
        print(f"  jumps >{th}: {int((d1>th).sum())}  ({(d1>th).sum()/(len(x)/fs):.1f}/s)")
    print(f"  max jump: {d1.max():.0f}")
    idx = np.where(d1 > 12000)[0]
    if len(idx) > 3:
        iv = np.diff(idx) / fs * 1000
        print(f"  big-jump interval ms: median {np.median(iv):.1f}  <2ms fraction "
              f"{float((iv<2).mean())*100:.0f}%")
    i0 = min(int(np.argmax(wr)) * win, len(x) - fs)
    seg = x[i0:i0+fs] * np.hanning(fs)
    S = np.abs(np.fft.rfft(seg))**2; f = np.fft.rfftfreq(fs, 1/fs); tot = S.sum()
    print("  bands:", {f"{lo//1000}-{hi//1000}k": round(float(S[(f>=lo)&(f<hi)].sum()/tot*100), 2)
                        for lo, hi in ((0,2000),(2000,5000),(5000,10000),(10000,20000),(20000,24000))})

if __name__ == "__main__":
    main()
