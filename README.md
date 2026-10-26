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

## Hardware Requirements

-   **Microcontroller:**  SAMD51-based Arduino-compatible board (e.g. Adafruit Metro M4 Express, ItsyBitsy M4)
-   **Controls:**  Potentiometers or CV sources for:
    -   Resonance (A2)
    -   Cutoff (A3)
    -   Envelope Modulation (A5)
    -   Decay (A4)
    -   Accent (A1)
-   **Audio Output:**  Use A0 (onboard DAC if available) or an external DAC (see below)
-   **Power:**  3.3V or 5V (check your board’s requirements)

## Audio Output Options

**1. Onboard DAC (Recommended for Simplicity)**

-   Many SAMD51 boards provide a 12-bit DAC on A0.
-   Use `analogWrite(A0, value);`  in the code.
-   Add a simple RC low-pass filter (e.g., 1kΩ resistor in series with output, 10nF capacitor from output to GND) for audio smoothing.

**2. External DAC (For Higher Quality)**

-   [MCP4725 (I2C)](https://www.adafruit.com/product/935)  or [MCP4921 (SPI)](https://www.adafruit.com/product/882)
-   Wire VCC, GND, SDA/SCL (I2C) or MOSI/SCK/CS (SPI) as per your DAC and board.
-   Use the appropriate Arduino library (e.g., [Adafruit_MCP4725](https://github.com/adafruit/Adafruit_MCP4725)).
-   Replace all `analogWrite(A0, ...)`  calls with your DAC’s output function.
-   Note: ItsyBitsy M4 doesn't support i2s DACs like PCM5102 [source](https://www.mouser.com/pdfdocs/introducing-adafruit-itsybitsy-m4.pdf?srsltid=AfmBOopDvCFJST56uh57h6QK3D61dIotFfml4tWEUYYRJc_unQqp5E1G)

## Software Requirements

-   **Arduino IDE**  (or PlatformIO)
-   **Board Support Package:**  Install “Adafruit SAMD Boards” via the Arduino Boards Manager.
-   **Libraries:**
    -   `SAMD51_InterruptTimer.h`  (or a compatible timer interrupt library, e.g., [Adafruit_ZeroTimer](https://github.com/adafruit/Adafruit_ZeroTimer))
    -   For external DAC: corresponding libraries (see above)
-   **Required Files:**  Ensure these header files (lookup tables) are in the same directory as `303_m4_fusion.ino`:
    -   `303filt_tanh.h`
    -   `sinetable.h`
    -   `midi_note_table_50x64.h`
    -   `303filt_coeff_5us.h`
    -   `303_vcf_decay_table.h`

## Setup and Usage

1.  **Connect Controls:**  
    Wire potentiometers (or CV) to the analog pins as listed in Hardware Requirements.
2.  **Audio Out:**  
    Connect A0 (with a low-pass filter) to an amplifier or audio interface.
3.  **Install Libraries:**  
    Install any required libraries via Arduino Library Manager or PlatformIO.
4.  **Open and Compile:**  
    Open `303_m4_fusion.ino`  in Arduino IDE.
5.  **Upload:**  
    Select your board and port, compile, and upload.
6.  **Play:**  
    The built-in sequencer will play a preset pattern; adjust the pots for classic 303 tweaking.

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

-   **Patterns:**  
    Edit the `sequence[]`  array for your own sequences (see comments in code).
-   **Filter/Envelope Behavior:**  
    Adjust coefficients and tables for different response curves or time constants.
-   **External Control:**  
    Modify `loop()`  to add MIDI or CV/Gate input for more flexible sequencing.

## References

-   [Roland TB-303](https://en.wikipedia.org/wiki/Roland_TB-303)  background
-   [Adafruit Metro M4 Express](https://www.adafruit.com/product/3382)
-   [Adafruit MCP4725 DAC](https://www.adafruit.com/product/935)
