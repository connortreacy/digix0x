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
#include "controls_latch.h"
// Control order: 0 accent, 1 reso, 2 cutoff, 3 decay, 4 env-mod.
static ControlLatch g_latch[5];
static const uint8_t g_latch_cc[5] = {MIDI_CC_ACCENT, MIDI_CC_RESO, MIDI_CC_CUTOFF,
                                      MIDI_CC_DECAY, MIDI_CC_ENVMOD};
static int8_t g_cc_wave = -1;   // CC waveform override (-1 = follow switch)
static uint8_t g_last_sw = 0xff;
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
        for (int i = 0; i < 5; i++)
          if (e.cc == g_latch_cc[i]) latch_cc(&g_latch[i], (uint16_t)(e.value << 3));
        if (e.cc == MIDI_CC_WAVEFORM) g_cc_wave = e.value >= 64 ? 1 : 0;
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
  c.accent = latch_pot(&g_latch[0], c.accent);
  c.reso = latch_pot(&g_latch[1], c.reso);
  c.cutoff = latch_pot(&g_latch[2], c.cutoff);
  c.decay = latch_pot(&g_latch[3], c.decay);
  c.env_mod = latch_pot(&g_latch[4], c.env_mod);
  if (c.waveform != g_last_sw) { g_last_sw = c.waveform; g_cc_wave = -1; }  // switch re-takes
  if (g_cc_wave >= 0) c.waveform = (uint8_t)g_cc_wave;
  tb303_set_controls(&synth, c.accent, c.reso, c.cutoff, c.decay, c.env_mod, c.waveform);
#else
  // ESP32: audio task (Core 1) drains the queue and applies controls;
  // loop only feeds the parsers and must not block the IDLE task.
  delay(1);
#endif
}
