#pragma once
// TB-303 DSP core: pure C99, no Arduino/driver dependencies.
// Transcribed from the SAMD51 myISR()/loop() in main (bec716c) with the
// MIDI note handling merged from midi-current (d23eb96). Constants and
// update order are preserved so the sound matches the M4 build.
// One core instance renders one mono sample per tb303_render() call at
// CONFIG_SAMPLE_RATE (default 50 kHz; filter tables assume 5 us steps).
#include <stdint.h>
#include <string.h>
#include "config.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct {
  // VCO
  uint32_t saw; uint16_t freq_offset, threshold, freq_inv, sq_note;
  // filter caps
  float cap1, cap2, cap3, cap4, cap_out, cap_reso, cap_vca1, cap_vca2, cap_reso2;
  float k0, k1, k2, k3, k4, k5, k6, k10, k7, k8;
  // VCA env
  float vca_env, vca_env_decay, vca_delay_cap;
  uint8_t vca_env_timer1, vca_env_timer2, vca_env_state;
  // VCF env
  float vcf_env, vcf_env_decay; uint16_t vcf_env_timer1; uint8_t vcf_env_state;
  // accent
  float accent, accent_cap1, accent_cap2, accent_vca, accent_vcf;
  float k11, k12, k13, k14, k15;
  // slide
  float slide_cap, slide_resistor; uint8_t slide_timer;
  // controls (derived from pots/CC)
  float resonance, average, last_average, accent_amount, env_mod, cutoff;
  uint16_t pot_decay; uint8_t waveform; int16_t tune;
  uint8_t timer2;
  // sample-rate derived constants (see tb303_init; REF = 50 kHz tuning)
  float rate_scale;          // 50000 / CONFIG_SAMPLE_RATE
  float vcf_atk_alpha, accent_lp_attack, accent_lp_decay;
  float vca_delay_decay, vca_charge_inc, vca_atk_mul, vca_atk_inc;
  float vca_release_mul, vca_decay0, vcf_decay_accent;
  uint16_t env_div;          // envelope sub-rate divisor (10 at 50 kHz)
  uint32_t tempo_base;       // sequencer counts per sub-step (500 at 50 kHz)
  // sequencer / note state
  uint32_t tempo_counter, freq; uint8_t step_counter, current_step;
  uint8_t current_note; int16_t current_note_value;
  const uint8_t *sequence; uint8_t sequence_length;
  // external MIDI state
  uint8_t midi_note, prev_midi_note, pulse_counter; uint8_t ext_clock;
  uint8_t ext_control;  // latched: external notes/clock own the engine (internal seq off)
  uint8_t pulses_per_step, pulses_per_note;
} TB303;

void    tb303_init(TB303 *s);
void    tb303_set_sequence(TB303 *s, const uint8_t *seq, uint8_t len);
void    tb303_set_controls(TB303 *s, uint16_t accent, uint16_t reso,
                           uint16_t cutoff, uint16_t decay, uint16_t env_mod,
                           uint8_t waveform);
void    tb303_note_on(TB303 *s, uint8_t midi_note, uint8_t velocity);
void    tb303_note_off(TB303 *s, uint8_t midi_note);
void    tb303_pitch_bend(TB303 *s, int16_t bend);  // -8192..8191
void    tb303_clock(TB303 *s, uint8_t msg);        // 0xF8 tick, 0xFA start, 0xFC stop
int16_t tb303_render(TB303 *s);                    // one sample, legacy 12-bit DAC scale
float   tb303_render_float(TB303 *s);              // one sample, -1..1 full resolution (I2S/sim)
int32_t tb303_render_vco(TB303 *s);                // raw oscillator only (validation)

#if defined(__cplusplus)
}
#endif
