#!/usr/bin/env python3
"""Validate emulated VCO against hardware captures in ../303wavs/.

Refs are raw 303 VCO recordings (MIDI notes 12-76, 96 kHz float, ~10 s).
Renders come from `make vco` (host/vco_out/, packed notes 0-63, 50 kHz).
Filename offset between the two namings is derived from measured pitch,
not assumed. Per matched pair we compare: pitch (cents), period-averaged
cycle shape (correlation / normalised RMS error after phase alignment),
and harmonic magnitudes (dB, harmonics under 10 kHz only - the refs were
Bessel-filtered at 18 kHz and the render runs at 50 kHz).
"""
import csv, glob, re, sys
import numpy as np
from scipy.io import wavfile

REF_DIR, OUT_DIR = "../303wavs", "vco_out"
CYCLE = 1024

def load_mono(path):
    fs, d = wavfile.read(path)
    if d.dtype == np.int16: d = d.astype(np.float64) / 32768.0
    elif d.dtype == np.int32: d = d.astype(np.float64) / 2147483648.0
    else: d = d.astype(np.float64)
    if d.ndim > 1: d = d[:, 0]
    return fs, d - d.mean()

def estimate_f0(fs, x):
    seg = x[len(x)//3: 2*len(x)//3]
    win = seg * np.hanning(len(seg))
    spec = np.abs(np.fft.rfft(win))
    freqs = np.fft.rfftfreq(len(seg), 1/fs)
    band = (freqs >= 10) & (freqs <= 1500)
    k = np.argmax(np.where(band, spec, 0))
    if 0 < k < len(spec)-1:  # parabolic interpolation (denominator < 0 at peak)
        a, b, c = spec[k-1], spec[k], spec[k+1]
        den = a - 2*b + c
        delta = 0.5*(a-c)/den if abs(den) > 1e-20 else 0.0
        k = k + max(-1.0, min(1.0, delta))
    return k * fs / len(seg)

def avg_cycle(fs, x, f0):
    period = fs / f0
    n = int(round(period))
    if n < 8: raise ValueError(f"period too short: {period:.1f} samples")
    seg = x[len(x)//4: 3*len(x)//4]
    idx = np.where((seg[:-1] < 0) & (seg[1:] >= 0))[0]  # rising crossings
    grid = np.linspace(0, 1, CYCLE, endpoint=False)
    t = np.linspace(0, 1, n, endpoint=False)
    cycles = [np.interp(grid, t, seg[s:s+n]) for s in idx if s + n < len(seg)][:200]
    if len(cycles) < 5: raise ValueError("too few cycles")
    c = np.mean(cycles, axis=0)
    return c / (np.sqrt(np.mean(c**2)) + 1e-20)

def align(ref, emu):
    # circular shift of emu maximising correlation with ref
    corr = np.fft.ifft(np.fft.fft(emu) * np.conj(np.fft.fft(ref))).real
    return np.roll(emu, -int(np.argmax(corr)))

def harmonics(cyc, f0, max_hz=10000):
    mag = np.abs(np.fft.rfft(cyc))[:21]
    mag = mag / (mag[1] + 1e-20)
    n = min(20, int(max_hz / f0))
    return 20*np.log10(mag[1:n+1] + 1e-9)

def cents(a, b): return 1200*np.log2(a/b) if a > 0 and b > 0 else float('nan')

def main():
    refs = {}   # (wave, note) -> (f0, cycle)
    for path in sorted(glob.glob(f"{REF_DIR}/saw*.wav") + glob.glob(f"{REF_DIR}/sq*.wav")):
        m = re.search(r'(saw|sq)(\d+)\.wav$', path)
        if not m or '_avg' in path or '_output' in path: continue
        fs, x = load_mono(path)
        f0 = estimate_f0(fs, x)
        try: cyc = avg_cycle(fs, x, f0)
        except ValueError: continue
        refs[(m.group(1), int(m.group(2)))] = (f0, cyc)
    print(f"loaded {len(refs)} hardware captures")

    # Derive filename offset from pitch: rendered packed p vs ref note n.
    votes = {}
    emu_f0 = {}
    for wave in ('saw', 'sq'):
        for p in range(64):
            fs, x = load_mono(f"{OUT_DIR}/vco_{wave}_p{p:02d}.wav")
            f0 = estimate_f0(fs, x)
            emu_f0[(wave, p)] = f0
            best = min((abs(cents(f0, rf0)), n) for (w, n), (rf0, _) in refs.items() if w == wave)
            votes.setdefault(best[1] - p, 0)
            votes[best[1] - p] += 1
    offset = max(votes, key=votes.get)
    print(f"pitch-derived offset: ref_note = packed + {offset} (votes {votes[offset]}/{sum(votes.values())})")

    rows, fails = [], []
    for (wave, p), f0e in sorted(emu_f0.items()):
        key = (wave, p + offset)
        if key not in refs: continue
        f0r, cref = refs[key]
        fs, x = load_mono(f"{OUT_DIR}/vco_{wave}_p{p:02d}.wav")
        try: cemu = align(cref, avg_cycle(fs, x, f0e))
        except ValueError: continue
        corr = float(np.corrcoef(cref, cemu)[0, 1])
        nrmse = float(np.sqrt(np.mean((cref - cemu)**2)))
        hr, he = harmonics(cref, f0r), harmonics(cemu, f0e)
        herr = float(np.mean(np.abs(hr[:len(he)] - he))) if len(he) else float('nan')
        perr = float(cents(f0e, f0r))
        rows.append(dict(wave=wave, packed=p, ref_note=key[1], pitch_err_cents=round(perr, 1),
                         cycle_corr=round(corr, 4), cycle_nrmse=round(nrmse, 4),
                         harm_err_db=round(herr, 2)))
        if abs(perr) > 50 or corr < 0.95: fails.append(rows[-1])
    with open("vco_report.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    # Headline tuning check: pitch when driven by MIDI note m (= packed+23)
    # vs true pitch 440*2^((m-69)/12). Systematic offset here = calibration bug.
    midi_err = [cents(f0, 440*2**((p + 23 - 69)/12)) for (w, p), f0 in emu_f0.items()]
    print(f"MIDI-input pitch err cents: median {np.nanmedian(midi_err):.0f} "
          f"(systematic; ~-1000 = one octave/semitones flat)")
    pc = np.abs([r['pitch_err_cents'] for r in rows]); cc = [r['cycle_corr'] for r in rows]
    print(f"compared {len(rows)} pairs -> vco_report.csv")
    print(f"pitch |err| cents: median {np.median(pc):.1f}  max {pc.max():.1f}")
    print(f"cycle correlation: median {np.median(cc):.4f}  min {min(cc):.4f}")
    print(f"harmonic err dB:   median {np.median([r['harm_err_db'] for r in rows]):.2f}")
    print(f"outliers (>50 cents or corr<0.95): {len(fails)}")
    for r in fails[:15]: print("  ", r)
    return 1 if len(fails) > len(rows)//4 else 0

if __name__ == "__main__":
    sys.exit(main())
