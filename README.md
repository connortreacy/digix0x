*Maintaining and building on the code from http://wiki.openmusiclabs.com/wiki/Digix0x, as found at https://www.modwiggler.com/forum/viewtopic.php?t=289244*

# digix0x TB-303 Emulator

A digital emulation of the Roland TB-303 synthesizer. This project recreates the iconic 303 sound with real-time audio synthesis, envelope shaping, accent and slide features, and a step sequencer.

> **Current development (branch `feature/esp32-dual-target`):** the synth engine now runs on both the original SAMD51 M4 boards and ESP32 (classic/S3/C6 family) from one codebase, with MIDI I/O and I2S audio running concurrently with 5 analog pot inputs. Board selection is configuration-only — see [BUILDING.md](BUILDING.md). The original single-target M4 code remains on `main`.

## Features

-   Classic TB-303 style sawtooth oscillator (plus per-note square wavetable)
-   Four-pole resonant diode ladder filter emulation
-   Accent and slide (portamento) handling
-   Step sequencer with rest, accent, and slide flags
-   MIDI input: notes (velocity > 64 = accent, legato = slide), pitch bend, CC control of all 5 knobs, external clock/start/stop (24 ppqn) with internal-sequencer fallback
-   Real-time synthesis: timer ISR on SAMD51, I2S-DMA task on ESP32

## Listen

▶ **[test_sweep.wav](host/test_sweep.wav)** — the internal pattern rendered by the exact firmware engine (float output path, 50 kHz):

-   0:00–3.84 — hold (cutoff 50, resonance 50)
-   3.84–7.68 — cutoff sweep 0 → 100
-   7.68–9.60 — cutoff back to 50
-   9.60–13.44 — resonance sweep 0 → 100

Regenerate with the host harness (see [BUILDING.md](BUILDING.md)); play it live with keys in the Mac simulator (`make -C host sim`).

## Hardware Requirements

One codebase, several boards — selection is by build environment / `config.h` only:

| Board | Audio | MIDI | Notes |
|---|---|---|---|
| Adafruit Metro / ItsyBitsy M4 (SAMD51) | Onboard 12-bit DAC (A0) and/or I2S | USB (MIDIUSB) + UART (Serial1) | The original target |
| ESP32 (WROOM DevKit) | I2S | UART | No USB device port |
| ESP32-S3 DevKit | I2S | UART + native USB | Planned final form |
| ESP32-C6 DevKit | I2S | UART | Single-core; no USB MIDI (USB is debug/serial) |
| Mac (simulator) | CoreAudio | Keyboard + CoreMIDI | No hardware needed — see below |

-   **Controls:** 5 potentiometers (or CV sources) for Accent / Resonance / Cutoff / Decay / Envelope Modulation, sampled concurrently with audio and MIDI. SAMD pins A1–A5; ESP32 ADC1 pins auto-selected per chip in `config.h`. Plus a saw/square toggle switch.
-   **MIDI wiring:** DIN/TRS via the standard optocoupler input circuit into the board UART (31250 baud). USB MIDI where the chip supports it.
-   **Power:**  3.3V logic throughout (check your board’s requirements).

## Audio Output

**1. I2S DAC (recommended)** — any config-free I2S DAC at 50 kHz, 32-bit frames: **PCM5102A** breakout (line-level, target part), MAX98357A (speaker amp, bench use), UDA1334A. No MCLK or register setup needed; PLL parts lock to the non-standard 50 kHz. I2S receives the full-resolution float render (the 12-bit path below measures ~27 dB more quantization hiss — A/B files: `host/compare_12bit.wav` vs `host/compare_32bit.wav`, rendered by the host harness).

**2. Onboard DAC (SAMD51 only)** — 12-bit DAC on A0 with a simple RC low-pass (e.g. 1kΩ series + 10nF to GND). Zero extra parts; this is the legacy path and its output value is bit-identical to the original code.

DACs needing I2C register init (PCM5122, ES9038, WM8731) are not yet supported — see Future Work. Note: ItsyBitsy M4 doesn't support I2S DACs like PCM5102 [source](https://www.mouser.com/pdfdocs/introducing-adafruit-itsybitsy-m4.pdf?srsltid=AfmBOopDvCFJST56uh57h6QK3D61dIotFfml4tWEUYYRJc_unQqp5E1G) — use its onboard DAC.

## Software Requirements

-   **PlatformIO** (recommended): `platformio.ini` provides `esp32-c6`, `esp32-s3`, `esp32` and `metro_m4` environments; all verified building. Libraries resolve automatically.
-   **Arduino IDE** also works for the SAMD build: “Adafruit SAMD Boards” via Boards Manager, plus `SAMD51_InterruptTimer`, `MIDIUSB`, and (only for I2S output) Adafruit Zero I2S. Open `digix0x.ino`.
-   **Host tools (Mac/Linux):** a C compiler for the simulator/tests; Python 3 + numpy/scipy in `.venv` for the hardware-validation scripts.

All lookup tables and sources must stay together in the sketch folder (`303filt_*.h`, `midi_note_table_48k_invert3.h`, `saw_edge2.h`, `sqwavs.h`, `sinetable.h`, `tb303_core.c`, …).

## Setup and Usage

**No hardware (Mac simulator):**
```
make -C host sim && ./host/sim
```
Realtime playback with keyboard piano, knob keys, a 16-step sequencer/editor and CoreMIDI input — see [BUILDING.md](BUILDING.md) for the key map.

**ESP32:**
```
pio run -e esp32-c6 -t upload   # or esp32-s3 / esp32
```
Wire the I2S DAC and up to 5 pots per `config.h` (defaults per chip are in that file), connect DIN MIDI via optocoupler, power up — the internal pattern plays immediately; send MIDI clock/notes from a DAW to take over.

**SAMD51:** open `digix0x.ino` in Arduino IDE (or `pio run -e metro_m4 -t upload`), select your M4 board, upload. Pots on A1–A5, audio from A0 and/or I2S per `config.h`.

Full wiring, build and validation instructions: [BUILDING.md](BUILDING.md).

## How It Works

-   The DSP engine is platform-free C99 in `tb303_core.c`: oscillator, diode ladder filter, envelope generators, accent, slide, sequencer and MIDI note/clock handling. It renders one sample per call at 50 kHz (filter internally oversampled 4× with 5 µs steps).
-   Hardware access goes through a small HAL (`platform.h`): `platform_samd51.h` (timer ISR + DAC/I2S + USB/UART MIDI) and `platform_esp32.h` (I2S-DMA audio task + UART/USB MIDI + ADC pots). I/O selection and pin maps live in `config.h`.
-   Patterns can be edited in the pattern headers (`everybody303.h`, `default.h`, `riser.h`) or replaced at runtime via `tb303_set_sequence()`.

## Hardware Validation (303wavs)

The `303wavs/` folder holds raw VCO captures from a real 303 (saw + square, MIDI notes 12–76, 96 kHz). `host/validate_vco.py` renders our oscillator for every note (`make -C host validate`) and compares pitch, period-averaged waveform shape, and harmonic content:

-   Waveform correlation vs hardware: **median 0.9985, minimum 0.987** across 108 matched notes; harmonic error median **0.21 dB**. The oscillator emulation is solid.
-   The comparison found a real bug: MIDI input played **~1030 cents flat** (the inherited `-23` note offset). Now fixed (`MIDI_NOTE_OFFSET 13` + an 18-step tuning trim in `tb303_core.c`); MIDI pitch error is now **median −2 cents**, guarded by the regression test `host/test_tuning.c` (`make -C host test`).
-   Note the note table stores `(increment, inverse)` pairs — tuning offsets must stay even or pitch inverts. See comments in `tb303_core.c`.
-   Through-filter comparison is still open: the captures are VCO-only, so filter validation awaits full-synth hardware recordings (play the `host/test_midi.wav` phrase on both, dial the hardware knobs to match, record, compare).

## Future Work

-   **Sample-rate move (48/96 kHz):** currently 50 kHz because the filter coefficient table (`303filt_coeff_5us.h`, 1024 cutoff steps × 6 gains) is tuned for 5 µs steps. Moving rates means rescaling that table by `Ts_new/Ts_old` (gains are linear in `Ts`) **plus** every other per-sample constant: note-table increments, decay multipliers (`m_new = m_old^(Fs_old/Fs_new)`), accent/slide gains, envelope timer counts, sequencer tempo. Recommended approach: refactor the core to derive constants from sample rate at init, then golden-master A/B against 50 kHz renders and re-run the 303wavs validation. Standard rates only matter for fixed-rate codecs/USB audio; PLL DACs (e.g. PCM5102A) lock at 50 kHz today.
-   **ESP32-S3 as final form:** native USB MIDI (already gated in code), dual core at 240 MHz. ESP32-C6 works now via UART MIDI (no USB device peripheral; BLE MIDI is a possible later addition).
-   **Configurable I/O growth:** DACs needing register setup (PCM5122, ES9038, WM8731) need an I2C init path in the HAL.
-   **Filter hardware captures** for through-filter validation (see above).

## Customization

-   **Patterns:** edit the pattern headers (`everybody303.h` is the default; `default.h`, `riser.h` are alternates) or call `tb303_set_sequence()` at runtime. Pattern bytes are `note | accent<<6 | slide<<7`, note 0 = rest.
-   **I/O selection:** everything lives in `config.h` — audio out (DAC / I2S / both), MIDI sources (USB / UART), sample rate, and all pin assignments (ESP32 defaults auto-select per chip).
-   **MIDI control:** CC 71/74/75/76/77 drive Resonance/Cutoff/Env-Mod/Decay/Accent until the matching physical pot moves; velocity > 64 accents; overlapping notes slide.
-   **Filter/envelope behaviour:** coefficients and tables are in `303filt_coeff_5us.h` and `303_vcf_decay_table.h` — see the tuning/validation notes above before touching them.
-   **Trying sounds without hardware:** the Mac simulator (`host/sim.c`) and offline renders (`make -C host test`) run the identical engine.

## References

-   [Roland TB-303](https://en.wikipedia.org/wiki/Roland_TB-303)  background
-   [Adafruit Metro M4 Express](https://www.adafruit.com/product/3382)
-   [BUILDING.md](BUILDING.md) — wiring, builds, simulator keys, validation workflow
