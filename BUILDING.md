# Building digix0x (branch: feature/esp32-dual-target)

One sketch, two targets. DSP is platform-free C in `tb303_core.c`;
all hardware access goes through `platform.h`. Configure I/O in `config.h`.

## Layout
- `digix0x.ino` — glue only (setup/loop), identical for both boards.
- `tb303_core.h/.c` — synth engine (osc, ladder filter, envs, accent, slide, sequencer, MIDI note/clock handling).
- `platform_samd51.h` — M4: TC 50 kHz ISR, DAC A0, optional I2S, USB MIDI (MIDIUSB), UART MIDI (Serial1), pots A1–A5.
- `platform_esp32.h` — ESP32: I2S DMA task on Core 1, UART MIDI (Serial2) + optional S2/S3 USB MIDI, 5× ADC1 pots, Core 0 polls controls/MIDI.
- `host/` — native build that renders test WAVs (`make -C host test`).

## M4 (Metro / ItsyBitsy M4) — unchanged workflow
Arduino IDE, Adafruit SAMD core, libraries `SAMD51_InterruptTimer`,
`MIDIUSB`, and `Adafruit ZeroI2S` only if `CONFIG_AUDIO_OUT` uses I2S.
Open `digix0x.ino`, select board, upload. Default: DAC + I2S + USB/UART MIDI.
Wiring per README (pots A1 accent, A2 reso, A3 cutoff, A4 decay, A5 env mod,
wave switch D12). UART MIDI: Serial1 TX/RX via standard optocoupler circuit.

## ESP32 (DevKit/WROOM; S3 also fine)
Arduino IDE with ESP32 core (or PlatformIO `esp32dev`). No extra libraries.
Defaults in `config.h`: I2S out (BCK 26, WS 25, DOUT 22 → MAX98357A or
PCM5102A), UART MIDI (RX 16, TX 17, 31250 baud via optocoupler),
pots on ADC1 GPIO 36/39/34/35/32 (accent/reso/cutoff/decay/env mod),
wave switch GPIO 27. Audio runs on Core 1; MIDI+pots on Core 0.
Note: classic ESP32 has no USB device port — USB MIDI needs an S2/S3 board.
Sample rate is 50 kHz (`CONFIG_SAMPLE_RATE`); the I2S APLL is enabled so
non-standard rates clock cleanly. Filter tables assume this rate.

MIDI behaviour (both targets): velocity > 64 = accent, overlapping note =
slide (legato, no retrigger), external Start/Clock (24 ppqn) takes over the
internal sequencer, pitch bend ±2 semitones, CC 71/74/75/76/77 drive
reso/cutoff/env-mod/decay/accent until the matching pot moves.

## PlatformIO (recommended for ESP32)
`platformio.ini` builds the same sources per board — no code changes between envs:
`pio run -e esp32-c6` (verified building: RAM 6.4%, Flash 36.6%), `pio run -e esp32-s3`,
`pio run -e esp32`, `pio run -e metro_m4`. Add `-t upload` to flash. Pin defaults
auto-select per chip in `config.h`; the audio task auto-selects Core 1 on dual-core
parts and Core 0 on unicore parts (C2/C3/C6/H2). ESP32-C6 has no USB-MIDI device
support (UART MIDI only); S3 adds native USB MIDI.

## Validation
`make -C host test` renders test WAVs and runs the tuning regression test.
`make -C host validate` (needs `.venv` with numpy/scipy: `python3 -m venv .venv &&
.venv/bin/pip install numpy scipy`) compares the raw oscillator against the
hardware VCO captures in `../303wavs/` and writes `vco_report.csv`.
Latest results: waveform correlation median 0.9985 (min 0.987), harmonic error
median 0.21 dB, MIDI pitch error median −2 cents across matched notes.

## Host (no hardware)
`make -C host test` renders `test_sequence.wav` (internal pattern) and
`test_midi.wav` (accent/slide phrase) at 50 kHz. Use these as the reference
takes for A/B against the hardware 303: program the same phrase, set knobs
by ear/photo (they are not MIDI controllable), record, and compare spectrogram
+ envelope in your DAW. Calibration note: note-table index 832 measures
36.1 Hz; the `-23` MIDI offset is inherited from the original code, so check
absolute tuning against the hardware unit first and adjust the offset in
`tb303_note_on()` if needed.
