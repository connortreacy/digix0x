#pragma once
// ESP32 HAL: I2S DMA audio task pinned to Core 1 (block-paced, no timer
// jitter), MIDI + pots polled on Core 0 (Arduino loop task), shared state
// exchanged through a lock-free queue and a controls snapshot.
// Classic ESP32 has no native USB: CONFIG_MIDI_USB applies only to
// S2/S3 with the Arduino USB-MIDI class; UART MIDI always available.
#include "platform.h"
#ifdef PLATFORM_ESP32
#include <driver/i2s.h>
#if CONFIG_MIDI_USB && (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)) && __has_include(<USBMIDI.h>)
#include <USB.h>
#include <USBMIDI.h>
static USBMIDI usb_midi;
#define ESP32_USB_MIDI 1
#endif

#include "controls_latch.h"
static ControlLatch g_latch[5];
static const uint8_t g_latch_cc[5] = {MIDI_CC_ACCENT, MIDI_CC_RESO, MIDI_CC_CUTOFF,
                                      MIDI_CC_DECAY, MIDI_CC_ENVMOD};
static volatile uint16_t g_cc_shadow[128];
static volatile uint8_t g_cc_dirty[128];
static volatile int8_t g_cc_wave = -1;  // CC waveform override (-1 = follow switch)

static TB303 *g_synth;
static MidiEvent g_mq[64];
static volatile uint8_t g_mq_head, g_mq_tail;
static Controls g_controls;
static volatile bool g_controls_dirty;

static void mq_push(const MidiEvent &e) {
  uint8_t n = (g_mq_head + 1) & 63;
  if (n != g_mq_tail) { g_mq[g_mq_head] = e; g_mq_head = n; }
}
bool platform_midi_next(MidiEvent *e) {
  if (g_mq_tail == g_mq_head) return false;
  *e = g_mq[g_mq_tail]; g_mq_tail = (g_mq_tail + 1) & 63; return true;
}

// Running-status UART MIDI parser (shared shape with SAMD backend).
static uint8_t uart_status, uart_data[2], uart_count;
static void uart_feed(uint8_t b) {
  if (b >= 0xF8) {
    if (b > 0xFC) return;
    MidiEvent e = {}; e.type = b == 0xF8 ? MIDI_EV_CLOCK : b == 0xFA ? MIDI_EV_START : MIDI_EV_STOP;
    mq_push(e); return;
  }
  if (b & 0x80) { uart_status = b; uart_count = 0; return; }
  if (!uart_status) return;
  uart_data[uart_count++] = b;
  if (uart_count < (((uart_status & 0xE0) == 0xC0) ? 1 : 2)) return;
  uart_count = 0;
  MidiEvent e = {}; uint8_t cmd = uart_status & 0xF0;
  if (cmd == 0x90 && uart_data[1] > 0) { e.type = MIDI_EV_NOTE_ON; e.note = uart_data[0]; e.velocity = uart_data[1]; }
  else if (cmd == 0x80 || cmd == 0x90) { e.type = MIDI_EV_NOTE_OFF; e.note = uart_data[0]; }
  else if (cmd == 0xE0) { e.type = MIDI_EV_PITCH_BEND; e.value = (int16_t)((uart_data[1] << 7) | uart_data[0]) - 8192; }
  else if (cmd == 0xB0) { e.type = MIDI_EV_CC; e.cc = uart_data[0]; e.value = uart_data[1]; }
  else return;
  mq_push(e);
}

// Audio task: render blocks and pace on I2S DMA. Applies queued MIDI
// events and control snapshots between blocks (max added latency = one
// block = 64 frames = 1.28 ms at 50 kHz; DMA queue adds ~2 blocks).
static void audio_task(void *) {
  static int32_t buf[64 * 2];
  for (;;) {
    MidiEvent e;
    while (platform_midi_next(&e)) {
      switch (e.type) {
        case MIDI_EV_NOTE_ON: tb303_note_on(g_synth, e.note, e.velocity); break;
        case MIDI_EV_NOTE_OFF: tb303_note_off(g_synth, e.note); break;
        case MIDI_EV_PITCH_BEND: tb303_pitch_bend(g_synth, e.value); break;
        case MIDI_EV_CLOCK: tb303_clock(g_synth, 0xF8); break;
        case MIDI_EV_START: tb303_clock(g_synth, 0xFA); break;
        case MIDI_EV_STOP: tb303_clock(g_synth, 0xFC); break;
        case MIDI_EV_CC:  // staged for the latch in platform_poll_midi()
          if (e.cc == MIDI_CC_WAVEFORM) g_cc_wave = e.value >= 64 ? 1 : 0;
          else if (e.cc < 128) { g_cc_shadow[e.cc] = (uint16_t)(e.value << 3); g_cc_dirty[e.cc] = 1; }
          break;
        default: break;
      }
    }
    if (g_controls_dirty) {
      g_controls_dirty = false;
      tb303_set_controls(g_synth, g_controls.accent, g_controls.reso,
                         g_controls.cutoff, g_controls.decay, g_controls.env_mod,
                         g_controls.waveform);
    }
    for (int i = 0; i < 64; i++) {
      // Full-resolution float path: the 12-bit DAC render would cap SNR ~74 dB.
      int32_t s = (int32_t)(tb303_render_float(g_synth) * 2147483647.0f);
      buf[2 * i] = s; buf[2 * i + 1] = s;
    }
    size_t written;
    i2s_write(I2S_NUM_0, buf, sizeof(buf), &written, portMAX_DELAY);
  }
}

void platform_setup(TB303 *synth) {
  g_synth = synth;
  pinMode(ESP32_PIN_WAVE_SW, INPUT_PULLUP);
  const int pots[5] = {ESP32_PIN_POT_ACCENT, ESP32_PIN_POT_RESO, ESP32_PIN_POT_CUTOFF,
                       ESP32_PIN_POT_DECAY, ESP32_PIN_POT_ENVMOD};
  for (int i = 0; i < 5; i++) pinMode(pots[i], INPUT);
  analogReadResolution(10);
  analogSetAttenuation(ADC_11db);  // full 0..3.3 V pot range

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = CONFIG_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.dma_buf_count = 3; cfg.dma_buf_len = 64;  // ~1.3 ms/buffer at 50 kHz
  cfg.use_apll = true;  // cleaner clock at non-standard 50 kHz
  i2s_pin_config_t pins = {ESP32_PIN_I2S_BCK, ESP32_PIN_I2S_WS, ESP32_PIN_I2S_DOUT, -1, -1};
  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &pins);

#if CONFIG_MIDI_UART
  Serial2.begin(31250, SERIAL_8N1, ESP32_PIN_MIDI_RX, ESP32_PIN_MIDI_TX);
#endif
#ifdef ESP32_USB_MIDI
  usb_midi.begin(); USB.begin();
#endif
  // Core 1 on dual-core parts (ESP32/S3); Core 0 on unicore (C2/C3/C6/H2),
  // where the I2S-DMA-paced task still preempts MIDI/control work cleanly.
  const BaseType_t audio_core = portNUM_PROCESSORS > 1 ? 1 : 0;
  xTaskCreatePinnedToCore(audio_task, "tb303-audio", 8192, nullptr, 24, nullptr, audio_core);
}

// Map one USB-MIDI packet (same layout as SAMD MIDIUSB) into the queue.
#ifdef ESP32_USB_MIDI
static void usb_packet_feed(uint8_t b1, uint8_t b2, uint8_t b3) {
  MidiEvent e = {}; uint8_t cmd = b1 & 0xF0;
  if (b1 == 0xF8) e.type = MIDI_EV_CLOCK;
  else if (b1 == 0xFA) e.type = MIDI_EV_START;
  else if (b1 == 0xFC) e.type = MIDI_EV_STOP;
  else if (cmd == 0x90 && b3 > 0) { e.type = MIDI_EV_NOTE_ON; e.note = b2; e.velocity = b3; }
  else if (cmd == 0x80 || cmd == 0x90) { e.type = MIDI_EV_NOTE_OFF; e.note = b2; e.velocity = b3; }
  else if (cmd == 0xE0) { e.type = MIDI_EV_PITCH_BEND; e.value = (int16_t)((b3 << 7) | b2) - 8192; }
  else if (cmd == 0xB0) { e.type = MIDI_EV_CC; e.cc = b2; e.value = b3; }
  else return;
  mq_push(e);
}
#endif

void platform_poll_midi() {  // runs on Core 0 from loop()
#if CONFIG_MIDI_UART
  while (Serial2.available()) uart_feed((uint8_t)Serial2.read());
#endif
#ifdef ESP32_USB_MIDI
  midiEventPacket_t p;
  while (usb_midi.readPacket(&p)) usb_packet_feed(p.byte1, p.byte2, p.byte3);
#endif
  // 1 kHz control scan with 1-pole smoothing (ESP32 ADC is noisy).
  static uint32_t last_ms;
  if (millis() - last_ms < 1) return;
  last_ms = millis();
  static float sm[5];
  const int pins[5] = {ESP32_PIN_POT_ACCENT, ESP32_PIN_POT_RESO, ESP32_PIN_POT_CUTOFF,
                       ESP32_PIN_POT_DECAY, ESP32_PIN_POT_ENVMOD};
  uint16_t v[5];
  for (int i = 0; i < 5; i++) {
    sm[i] += 0.25f * ((float)analogRead(pins[i]) - sm[i]);
    v[i] = (uint16_t)(sm[i] + 0.5f);
  }
  for (int i = 0; i < 5; i++)  // hand newly staged CC values to the latch
    if (g_cc_dirty[g_latch_cc[i]]) {
      g_cc_dirty[g_latch_cc[i]] = 0;
      latch_cc(&g_latch[i], g_cc_shadow[g_latch_cc[i]]);
    }
  v[0] = latch_pot(&g_latch[0], v[0]); v[1] = latch_pot(&g_latch[1], v[1]);
  v[2] = latch_pot(&g_latch[2], v[2]); v[3] = latch_pot(&g_latch[3], v[3]);
  v[4] = latch_pot(&g_latch[4], v[4]);
  static uint8_t last_sw = 0xff;
  uint8_t sw = (uint8_t)digitalRead(ESP32_PIN_WAVE_SW);
  if (sw != last_sw) { last_sw = sw; g_cc_wave = -1; }  // physical switch re-takes
  if (g_cc_wave >= 0) sw = (uint8_t)g_cc_wave;
  Controls c = {v[0], v[1], v[2], v[3], v[4], sw};
  // Field-wise compare: memcmp would also test struct padding bytes.
  if (c.accent != g_controls.accent || c.reso != g_controls.reso ||
      c.cutoff != g_controls.cutoff || c.decay != g_controls.decay ||
      c.env_mod != g_controls.env_mod || c.waveform != g_controls.waveform) {
    g_controls = c; g_controls_dirty = true;
  }
}

void platform_midi_send_clock(uint8_t msg) {
#if CONFIG_MIDI_UART
  Serial2.write(msg);
#endif
#ifdef ESP32_USB_MIDI
  midiEventPacket_t p = {0x0F, msg, 0, 0};  // CIN 0xF: single-byte realtime
  usb_midi.writePacket(&p);
#endif
}

Controls platform_read_controls() { return g_controls; }
#endif  // PLATFORM_ESP32
