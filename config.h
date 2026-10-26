#pragma once
// digix0x user configuration. Edit this file to select platform I/O.
// Defaults auto-detect the board; override any switch below as needed.

// ---------- Platform (normally auto-detected, rarely set by hand) ----------
// #define PLATFORM_SAMD51
// #define PLATFORM_ESP32
// #define PLATFORM_HOST

// ---------- Audio output: AUDIO_DAC, AUDIO_I2S, or AUDIO_DAC_I2S ----------
#define AUDIO_DAC   1
#define AUDIO_I2S   2
#define AUDIO_DAC_I2S 3
#ifndef CONFIG_AUDIO_OUT
#  if defined(ARDUINO_ARCH_ESP32)
#    define CONFIG_AUDIO_OUT AUDIO_I2S
#  else
#    define CONFIG_AUDIO_OUT AUDIO_DAC_I2S  // M4: DAC always, I2S if board wired
#  endif
#endif

// ---------- MIDI input/output ----------
// MIDI_SRC_USB / MIDI_SRC_UART may be combined (both listened to).
#define CONFIG_MIDI_USB  1   // SAMD51: MIDIUSB. ESP32-S2/S3: TinyUSB. Classic ESP32: ignored.
#define CONFIG_MIDI_UART 1   // 5-pin DIN / TRS via hardware UART at 31250 baud.
#define CONFIG_MIDI_ECHO_CLOCK 0  // forward received clock/start/stop to MIDI out.

// ---------- Sample rate ----------
// Filter tables (303filt_coeff_5us.h) are tuned for a 5 us filter step:
// 4 filter iterations per output sample at 50 kHz. Changing the rate
// detunes cutoff/envelope timing; keep 50000 unless you retune tables.
#ifndef CONFIG_SAMPLE_RATE
#  define CONFIG_SAMPLE_RATE 50000
#endif

// ---------- ESP32 pins (auto-selected per chip; override any below) ----------
// Defaults are chosen per IDF target so a board swap needs no code edits.
// ADC pins are always ADC1 (safe with radio off; we never enable WiFi/BT).
#if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C2)
#  define ESP32_PIN_I2S_BCK   21
#  define ESP32_PIN_I2S_WS    22
#  define ESP32_PIN_I2S_DOUT  23
#  define ESP32_PIN_MIDI_RX   16
#  define ESP32_PIN_MIDI_TX   17
#  define ESP32_PIN_WAVE_SW   10
#  define ESP32_PIN_POT_ACCENT 1
#  define ESP32_PIN_POT_RESO   2
#  define ESP32_PIN_POT_CUTOFF 3
#  define ESP32_PIN_POT_DECAY  4
#  define ESP32_PIN_POT_ENVMOD 5
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#  define ESP32_PIN_I2S_BCK   18
#  define ESP32_PIN_I2S_WS    19
#  define ESP32_PIN_I2S_DOUT  20
#  define ESP32_PIN_MIDI_RX   16
#  define ESP32_PIN_MIDI_TX   17
#  define ESP32_PIN_WAVE_SW   21
#  define ESP32_PIN_POT_ACCENT 4
#  define ESP32_PIN_POT_RESO   5
#  define ESP32_PIN_POT_CUTOFF 6
#  define ESP32_PIN_POT_DECAY  7
#  define ESP32_PIN_POT_ENVMOD 8
#else  // classic ESP32 (WROOM DevKit)
#  define ESP32_PIN_I2S_BCK   26
#  define ESP32_PIN_I2S_WS    25
#  define ESP32_PIN_I2S_DOUT  22
#  define ESP32_PIN_MIDI_RX   16
#  define ESP32_PIN_MIDI_TX   17
#  define ESP32_PIN_WAVE_SW   27
#  define ESP32_PIN_POT_ACCENT 36
#  define ESP32_PIN_POT_RESO   39
#  define ESP32_PIN_POT_CUTOFF 34
#  define ESP32_PIN_POT_DECAY  35
#  define ESP32_PIN_POT_ENVMOD 32
#endif

// ---------- SAMD51 pins (Metro / ItsyBitsy M4, matches README) ----------
#define SAMD_PIN_POT_ACCENT A1
#define SAMD_PIN_POT_RESO   A2
#define SAMD_PIN_POT_CUTOFF A3
#define SAMD_PIN_POT_DECAY  A4
#define SAMD_PIN_POT_ENVMOD A5
#define SAMD_PIN_WAVE_SW    12

// MIDI CC -> knob mapping (sent values 0..127 scale to pot range 0..1023).
// Pots take over again as soon as they move (simple jump takeover in v1).
#define MIDI_CC_RESO   71
#define MIDI_CC_CUTOFF 74
#define MIDI_CC_ENVMOD 75
#define MIDI_CC_DECAY  76
#define MIDI_CC_ACCENT 77
