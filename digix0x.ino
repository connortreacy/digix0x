// digix0x TB-303 emulator: dual-target build (SAMD51 M4 + ESP32).
// DSP lives in tb303_core.c (pure C, also built by host/ for testing).
// Hardware access lives in platform_samd51.h / platform_esp32.h.
// Select I/O in config.h. See BUILDING.md.
#include <Arduino.h>
#include "config.h"
#include "platform.h"
#include "tb303_core.h"

static TB303 synth;
#if defined(PLATFORM_SAMD51)
static uint16_t cc_override[128];
static bool cc_active[128];
#endif

#if defined(PLATFORM_SAMD51)
#include "platform_samd51.h"
#elif defined(PLATFORM_ESP32)
#include "platform_esp32.h"
#endif

void setup() {
  Serial.begin(115200);
  tb303_init(&synth);
  platform_setup(&synth);
}

void loop() {
  platform_poll_midi();
#if defined(PLATFORM_SAMD51)
  // SAMD: apply events/controls here; the timer ISR only renders.
  MidiEvent e;
  while (platform_midi_next(&e)) {
    switch (e.type) {
      case MIDI_EV_NOTE_ON: tb303_note_on(&synth, e.note, e.velocity); break;
      case MIDI_EV_NOTE_OFF: tb303_note_off(&synth, e.note); break;
      case MIDI_EV_PITCH_BEND: tb303_pitch_bend(&synth, e.value); break;
      case MIDI_EV_CLOCK: tb303_clock(&synth, 0xF8); break;
      case MIDI_EV_START: tb303_clock(&synth, 0xFA); break;
      case MIDI_EV_STOP: tb303_clock(&synth, 0xFC); break;
      case MIDI_EV_CC:
        if (e.cc < 128) { cc_override[e.cc] = (uint16_t)(e.value << 3); cc_active[e.cc] = true; }
        break;
      default: break;
    }
#if CONFIG_MIDI_ECHO_CLOCK
    if (e.type == MIDI_EV_CLOCK) platform_midi_send_clock(0xF8);
    if (e.type == MIDI_EV_START) platform_midi_send_clock(0xFA);
    if (e.type == MIDI_EV_STOP) platform_midi_send_clock(0xFC);
#endif
  }
  Controls c = platform_read_controls();
  if (cc_active[MIDI_CC_ACCENT]) c.accent = cc_override[MIDI_CC_ACCENT];
  if (cc_active[MIDI_CC_RESO]) c.reso = cc_override[MIDI_CC_RESO];
  if (cc_active[MIDI_CC_CUTOFF]) c.cutoff = cc_override[MIDI_CC_CUTOFF];
  if (cc_active[MIDI_CC_DECAY]) c.decay = cc_override[MIDI_CC_DECAY];
  if (cc_active[MIDI_CC_ENVMOD]) c.env_mod = cc_override[MIDI_CC_ENVMOD];
  tb303_set_controls(&synth, c.accent, c.reso, c.cutoff, c.decay, c.env_mod, c.waveform);
#else
  // ESP32: audio task (Core 1) drains the queue and applies controls;
  // loop only feeds the parsers and must not block the IDLE task.
  delay(1);
#endif
}
