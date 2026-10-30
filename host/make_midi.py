#!/usr/bin/env python3
"""Generate everybody303.mid from ../everybody303.h (the firmware pattern).
Packed byte = note | accent<<6 | slide<<7, note 0 = rest.
MIDI note = packed + 13 (MIDI_NOTE_OFFSET in tb303_core.c).
Slides overlap (note-on before note-off) so the core slides;
accents get velocity 110, normal notes 50. Panel CCs are written
at the start (Accent/Reso/Cutoff/Decay/EnvMod = 76/64/64/64/76).
Usage: make_midi.py [loops] [bpm] [outfile]"""
import re, struct, sys

PPQ = 480
STEP = PPQ // 4  # 16th note

def vlq(n):
    out = [n & 0x7F]; n >>= 7
    while n:
        out.append(0x80 | (n & 0x7F)); n >>= 7
    return bytes(reversed(out))

def main():
    loops = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    bpm = float(sys.argv[2]) if len(sys.argv) > 2 else 140.0
    out_path = sys.argv[3] if len(sys.argv) > 3 else "everybody303.mid"
    src = open("../everybody303.h").read()
    pattern = [int(a) | (int(b) << 6) | (int(c) << 7)
               for a, b, c in re.findall(r'(\d+)\s*\|\s*(\d+)<<6\s*\|\s*(\d+)\s*<<\s*7', src)]
    if len(pattern) != 16:
        pattern = [int(x) for x in re.findall(r'\b(\d+)\b', src)][:16]
    print(f"pattern ({len(pattern)} steps): {pattern}")

    events = []  # (tick, order, bytes); order: off(0) before on(1), slide on(-1) first
    def ev(tick, order, data): events.append((tick, order, data))
    # panel setup at tick 0
    for ccnum, val in ((77, 76), (71, 64), (74, 64), (76, 64), (75, 76)):
        ev(0, 0, bytes([0xB0, ccnum, val]))
    held = None
    for step_i in range(loops * 16):
        tick = step_i * STEP
        p = pattern[step_i % 16]
        note, accent, slide = p & 0x3F, (p >> 6) & 1, (p >> 7) & 1
        if note == 0:
            if held is not None:
                ev(tick, 0, bytes([0x80, held, 0])); held = None
            continue
        midi = note + 13
        if held is not None and not slide:
            ev(tick, 0, bytes([0x80, held, 0])); held = None
        ev(tick, -1 if slide else 1, bytes([0x90, midi, 110 if accent else 50]))
        if held is not None and slide:  # overlapping off, just after the on
            ev(tick + 12, 0, bytes([0x80, held, 0]))
        held = midi
        if not slide:  # gate off at 80% unless a later step sustains it
            nxt = pattern[(step_i + 1) % 16] if step_i + 1 < loops * 16 else 0
            if not (nxt >> 7 & 1):
                ev(tick + int(STEP * 0.8), 0, bytes([0x80, midi, 0])); held = None
    if held is not None:
        ev(loops * 16 * STEP, 0, bytes([0x80, held, 0]))
    events.sort(key=lambda e: (e[0], e[1]))

    track = bytearray()
    track += vlq(0) + b'\xff\x03\x0d' + b'everybody303 '
    tempo = int(60_000_000 / bpm)
    track += vlq(0) + b'\xff\x51\x03' + tempo.to_bytes(3, 'big')
    last = 0
    for tick, _, data in events:
        track += vlq(tick - last) + data; last = tick
    track += vlq(0) + b'\xff\x2f\x00'
    smf = b'MThd' + struct.pack('>IHHH', 6, 0, 1, PPQ)
    smf += b'MTrk' + struct.pack('>I', len(track)) + bytes(track)
    open(out_path, 'wb').write(smf)
    print(f"wrote {out_path}: {len(smf)} bytes, {loops} loops at {bpm:.0f} BPM, "
          f"{sum(1 for _,_,d in events if d[0]==0x90)} note-ons")

if __name__ == "__main__":
    main()
