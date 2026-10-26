#pragma once
// SAMD51 (Metro M4 / ItsyBitsy M4) HAL: TC timer tick at 50 kHz,
// onboard DAC on A0, optional I2S (Adafruit_ZeroI2S), USB MIDI (MIDIUSB),
// hardware-UART MIDI on Serial1, 5 pots + waveform switch.
#include "platform.h"
#ifdef PLATFORM_SAMD51
#include "SAMD51_InterruptTimer.h"
#if CONFIG_MIDI_USB
#include "MIDIUSB.h"
#endif
#if CONFIG_AUDIO_OUT != AUDIO_DAC
#include <Adafruit_ZeroI2S.h>
static Adafruit_ZeroI2S i2s_out;
#endif

static TB303 *g_synth;
static MidiEvent g_mq[32];
static volatile uint8_t g_mq_head, g_mq_tail;
static void mq_push(MidiEvent e) {
  uint8_t n = (g_mq_head + 1) & 31;
  if (n != g_mq_tail) { g_mq[g_mq_head] = e; g_mq_head = n; }
}
bool platform_midi_next(MidiEvent *e) {
  if (g_mq_tail == g_mq_head) return false;
  *e = g_mq[g_mq_tail]; g_mq_tail = (g_mq_tail + 1) & 31; return true;
}

// Running-status UART MIDI parser (Serial1, 31250 baud).
static uint8_t uart_status, uart_data[2], uart_count;
static void uart_feed(uint8_t b) {
  if (b >= 0xF8) {  // realtime: single byte, never disturbs running status
    MidiEvent e = {}; e.type = b == 0xF8 ? MIDI_EV_CLOCK : b == 0xFA ? MIDI_EV_START : MIDI_EV_STOP;
    if (b <= 0xFC) mq_push(e); return;
  }
  if (b & 0x80) { uart_status = b; uart_count = 0; return; }
  if (!uart_status) return;
  uart_data[uart_count++] = b;
  uint8_t need = ((uart_status & 0xE0) == 0xC0) ? 1 : 2;
  if (uart_count < need) return;
  uart_count = 0;
  MidiEvent e = {};
  uint8_t cmd = uart_status & 0xF0;
  if (cmd == 0x90 && uart_data[1] > 0) { e.type = MIDI_EV_NOTE_ON; e.note = uart_data[0]; e.velocity = uart_data[1]; }
  else if (cmd == 0x80 || cmd == 0x90) { e.type = MIDI_EV_NOTE_OFF; e.note = uart_data[0]; e.velocity = uart_data[1]; }
  else if (cmd == 0xE0) { e.type = MIDI_EV_PITCH_BEND; e.value = (int16_t)((uart_data[1] << 7) | uart_data[0]) - 8192; }
  else if (cmd == 0xB0) { e.type = MIDI_EV_CC; e.cc = uart_data[0]; e.value = uart_data[1]; }
  else return;
  mq_push(e);
}

static void samd_isr() {
  int16_t out = tb303_render(g_synth);
#if CONFIG_AUDIO_OUT != AUDIO_I2S
  int32_t dac = (int32_t)out - 0x0500;
  analogWrite(A0, dac < 0 ? 0 : dac > 4095 ? 4095 : dac);
#endif
#if CONFIG_AUDIO_OUT != AUDIO_DAC
  int32_t s = (int32_t)out << 15;
  i2s_out.write(s, s);
#endif
}

void platform_setup(TB303 *synth) {
  g_synth = synth;
  pinMode(13, OUTPUT); pinMode(SAMD_PIN_WAVE_SW, INPUT_PULLUP);
  analogReadResolution(10); analogWriteResolution(12);
#if CONFIG_MIDI_UART
  Serial1.begin(31250);
#endif
#if CONFIG_AUDIO_OUT != AUDIO_DAC
  i2s_out.begin(I2S_32_BIT, CONFIG_SAMPLE_RATE); i2s_out.enableTx();
#endif
  TC.startTimer(1000000 / CONFIG_SAMPLE_RATE, samd_isr);
}

void platform_poll_midi() {
#if CONFIG_MIDI_UART
  while (Serial1.available()) uart_feed((uint8_t)Serial1.read());
#endif
#if CONFIG_MIDI_USB
  midiEventPacket_t p;
  while ((p = MidiUSB.read()).header != 0) {
    MidiEvent e = {}; uint8_t cmd = p.byte1 & 0xF0;
    if (p.byte1 == 0xF8) e.type = MIDI_EV_CLOCK;
    else if (p.byte1 == 0xFA) e.type = MIDI_EV_START;
    else if (p.byte1 == 0xFC) e.type = MIDI_EV_STOP;
    else if (cmd == 0x90 && p.byte3 > 0) { e.type = MIDI_EV_NOTE_ON; e.note = p.byte2; e.velocity = p.byte3; }
    else if (cmd == 0x80 || cmd == 0x90) { e.type = MIDI_EV_NOTE_OFF; e.note = p.byte2; e.velocity = p.byte3; }
    else if (cmd == 0xE0) { e.type = MIDI_EV_PITCH_BEND; e.value = (int16_t)((p.byte3 << 7) | p.byte2) - 8192; }
    else if (cmd == 0xB0) { e.type = MIDI_EV_CC; e.cc = p.byte2; e.value = p.byte3; }
    else continue;
    mq_push(e);
  }
#endif
}

void platform_midi_send_clock(uint8_t msg) {
#if CONFIG_MIDI_UART
  Serial1.write(msg);
#endif
#if CONFIG_MIDI_USB
  midiEventPacket_t p = { (uint8_t)(msg == 0xF8 ? 0x0F : 0x0F), msg, 0, 0 };
  MidiUSB.sendMIDI(p); MidiUSB.flush();
#endif
}

Controls platform_read_controls() {
  Controls c;
  c.accent = analogRead(SAMD_PIN_POT_ACCENT);
  c.reso = analogRead(SAMD_PIN_POT_RESO);
  c.cutoff = analogRead(SAMD_PIN_POT_CUTOFF);
  c.decay = analogRead(SAMD_PIN_POT_DECAY);
  c.env_mod = analogRead(SAMD_PIN_POT_ENVMOD);
  c.waveform = digitalRead(SAMD_PIN_WAVE_SW);
  return c;
}
#endif  // PLATFORM_SAMD51
