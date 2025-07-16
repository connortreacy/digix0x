*Maintaining and building on the code from http://wiki.openmusiclabs.com/wiki/Digix0x, as found at https://www.modwiggler.com/forum/viewtopic.php?t=289244*

# digix0x TB-303 Emulator

A digital emulation of the Roland TB-303 synthesizer, running on SAMD51-based microcontrollers (e.g., Adafruit Metro M4/ItsyBitsy M4). This project recreates the iconic 303 sound with real-time audio synthesis, envelope shaping, accent and slide features, and a step sequencer.

## Features

-   Classic TB-303 style sawtooth oscillator
-   Four-pole resonant diode ladder filter emulation
-   Accent and slide (portamento) handling
-   Step sequencer with rest, accent, and slide flags
-   Real-time synthesis in a timer interrupt for authentic analog behavior

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

-   All sound synthesis and sequencing is performed in a high-speed timer interrupt (`myISR()`).
-   The code emulates the classic TB-303’s sawtooth oscillator, diode ladder filter, envelope generators, accent, and slide.
-   Patterns, slides, and accents are hardcoded in the `sequence[]`  array, but can be edited for new patterns.

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
