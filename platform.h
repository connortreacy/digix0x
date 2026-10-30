#pragma once
// Hardware abstraction layer. The DSP core (tb303_core.h) never touches
// Arduino, registers, or drivers; firmware glue talks to hardware only
// through these functions, one implementation per platform header.
#include <stdint.h>
#include "config.h"

#if defined(ARDUINO_ARCH_SAMD)
#  define PLATFORM_SAMD51 1
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#  define PLATFORM_ESP32 1
#elif !defined(PLATFORM_HOST)
#  define PLATFORM_HOST 1
#endif

// Normalised control values, 0..1023 (10-bit, matches legacy pot scaling).
typedef struct {
  uint16_t accent, reso, cutoff, decay, env_mod;
  uint8_t  waveform;  // 0 = saw, 1 = square
} Controls;

// Decoded MIDI events delivered to the core (channel already filtered).
typedef enum {
  MIDI_EV_NONE, MIDI_EV_NOTE_ON, MIDI_EV_NOTE_OFF,
  MIDI_EV_CLOCK, MIDI_EV_START, MIDI_EV_STOP,
  MIDI_EV_PITCH_BEND,  // value: -8192..8191
  MIDI_EV_CC           // cc, value
} MidiEventType;
typedef struct {
  MidiEventType type;
  uint8_t  note;     // MIDI note number
  uint8_t  velocity; // note on/off
  uint8_t  cc;
  int16_t  value;    // pitch bend or CC value
} MidiEvent;

// Implemented per platform (platform_*.h, included by digix0x.ino):
//   void     platform_setup(TB303*);            // clocks, pins, drivers, tick source
//   void     platform_poll_midi();              // fills midi queue
//   bool     platform_midi_next(MidiEvent*);    // pop one event
//   void     platform_midi_send_clock(uint8_t); // 0xF8/0xFA/0xFC out
//   Controls platform_read_controls();          // latest pot/switch state
// Tick source: SAMD51 = TC timer ISR; ESP32 = I2S-DMA task on Core 1.
// Both call tb303_render() once per output sample at CONFIG_SAMPLE_RATE.
