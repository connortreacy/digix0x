// TB-303 DSP core implementation. See tb303_core.h.
// Filter/envelope math transcribed verbatim from digix0x.ino (main bec716c);
// MIDI gate/accent/slide semantics from midi-current (d23eb96).
// Tables are tuned at REF_RATE; other CONFIG_SAMPLE_RATE values derive
// every per-sample constant from the rate at init (see tb303_init).
#include <math.h>
#include "tb303_core.h"

#define REF_RATE 50000.0f

static const uint16_t tanh_table[256] = {
#include "303filt_tanh.h"
};
static const uint16_t note_table[4096] = {
#include "midi_note_table_48k_invert3.h"
};
static const int16_t saw_edge[2048] = {
#include "saw_edge2.h"
};
static const float cv_table[6 * 1024] = {
#include "303filt_coeff_5us.h"
};
static float decay_table[1024] = {
#include "303_vcf_decay_table.h"
};
static const int16_t sq_wavs[64][1024] = {
#include "sqwavs.h"
};

#define HPF_Q 0.4f
#define HPF_W 8.0f

static const uint8_t default_sequence[16] = {
#include "everybody303.h"
};

// MIDI note -> packed 303 note offset. Was 23 in the original code;
// validation against hardware VCO captures (303wavs/, host/validate_vco.py)
// showed that plays ~1000 cents flat, while 13 lands within trim below.
#define MIDI_NOTE_OFFSET 13
// Residual tuning trim in note-table steps (64/semitone): hardware
// captures measure the table ~29 cents flat at 50 kHz, i.e. +19 steps.
// Must stay EVEN: the table stores (increment, inverse) pairs, so odd
// indices are inverse values, not increments.
#define TUNE_TRIM_STEPS 18

void tb303_init(TB303 *s) {
  memset(s, 0, sizeof(*s));
  // Rate derivation: integrator gains scale with step (x REF/Fs),
  // per-sample decay multipliers m become m^(REF/Fs), approach
  // coefficients a become 1-(1-a)^(REF/Fs), counts scale with Fs/REF.
  const float R = REF_RATE / (float)CONFIG_SAMPLE_RATE;
  s->rate_scale = R;
  s->env_div = (uint16_t)(10.0f / R + 0.5f); if (s->env_div < 1) s->env_div = 1;
  s->tempo_base = (uint32_t)(500.0f / R + 0.5f);
  s->vcf_atk_alpha = 1.0f - powf(0.85f, R);
  s->accent_lp_attack = 1.0f - powf(1.0f - 0.0404f, R);
  s->accent_lp_decay = 1.0f - powf(1.0f - 0.404f, R);
  s->vca_delay_decay = powf(0.9998f, R);
  s->vca_charge_inc = 0.002f * R;
  s->vca_atk_mul = powf(1.076f, R);
  s->vca_atk_inc = 0.0076f * R;
  s->vca_release_mul = powf(0.996f, R);
  s->vca_decay0 = powf(0.99991f, R);
  s->vcf_decay_accent = powf(0.9972f, R);
  const float ts = 1.0f / (4.0f * (float)CONFIG_SAMPLE_RATE);  // filter step
  s->k0 = 1.47f; s->k1 = 0.2876302f; s->k2 = 0.0797963f;
  s->k3 = 0.0654076f; s->k4 = 0.0072146f; s->k5 = 0.0013524f;
  s->k6 = ts * 6.283f * HPF_Q * HPF_W;
  s->k10 = ts * 6.283f * HPF_W / HPF_Q;
  s->k7 = 0.00785f * R; s->k8 = 0.0204f * R;
  s->vca_env_decay = s->vca_decay0; s->vcf_env_decay = s->vcf_decay_accent;
  s->k11 = 0.0018f; s->k12 = 0.00143f;
  s->k13 = s->k11 / 10.0f; s->k14 = s->k12 / 10.0f;
  s->slide_resistor = 1.0f - powf(1.0f - 0.00909f, R); s->slide_timer = 0xff;
  s->tune = TUNE_TRIM_STEPS;
  s->midi_note = 0xff; s->prev_midi_note = 0xff;  // 0xff = none (packed 0 is a note)
  s->freq = 100;
  s->sequence = default_sequence; s->sequence_length = 16;
  s->pulses_per_step = 6; s->pulses_per_note = 4;
}

void tb303_set_sequence(TB303 *s, const uint8_t *seq, uint8_t len) {
  s->sequence = seq; s->sequence_length = len; s->current_step = 0;
}

// Mirrors the pot->parameter scaling in loop() of both parent branches.
void tb303_set_controls(TB303 *s, uint16_t accent, uint16_t reso,
                        uint16_t cutoff, uint16_t decay, uint16_t env_mod,
                        uint8_t waveform) {
  s->accent_amount = 0.0009775f * accent;
  s->resonance = 0.005078125f * reso;
  uint16_t half = (uint16_t)(0x03ff - reso);
  s->k15 = 0.000385f * half;
  s->k14 = 0.000168f - 0.000000045f * half;
  s->k13 = 0.000165f - 0.0000000675f * half;
  s->k12 = 10.0f * s->k14; s->k11 = 10.0f * s->k13;
  // Accent-cap gains are per-sample integrator steps: scale with step size.
  // (k15 is a dimensionless mix ratio and stays unscaled.)
  s->k11 *= s->rate_scale; s->k12 *= s->rate_scale;
  s->k13 *= s->rate_scale; s->k14 *= s->rate_scale;
  s->cutoff = 0.332f * cutoff + 328.0f;
  s->env_mod = 0.16667f + 0.0008138f * env_mod;
  s->pot_decay = decay; s->waveform = waveform;
}

// force=1 (MIDI path): packed 0 is a real note (MIDI 13), not a rest.
// force=0 (sequencer/clock path): packed 0 encodes a rest, pitch holds.
static void apply_note(TB303 *s, uint8_t packed, uint8_t prev_packed, uint8_t force) {
  s->current_note = packed;
  if (force || (packed & 0x3f) != 0) s->current_note_value = (int16_t)((packed & 0x3f) << 6);
  if ((prev_packed & 0x80) == 0) {           // attack unless previous slid
    s->vca_env_state = 1; s->vcf_env_state = 1; s->slide_timer = 0xff;
    uint16_t tn = (uint16_t)(s->current_note_value + s->tune);
    if (tn > 4094) tn = 4094;
    s->sq_note = (uint16_t)((tn >> 6) & 0x003f);
    uint16_t interim = note_table[tn];
    s->freq = (uint32_t)((float)(interim << 10) * s->rate_scale);
    s->freq_offset = (uint16_t)(interim >> 3);
    s->threshold = (uint16_t)(interim >> 2);
    s->freq_inv = note_table[tn + 1];
    s->slide_cap = (float)s->current_note_value;
  } else {
    s->slide_timer = 0;                       // legato: slide, no retrigger
  }
  if ((packed & 0x40) == 0) {
    s->accent = 0; s->vcf_env_decay = powf(decay_table[s->pot_decay & 0x3ff], s->rate_scale);
  } else {
    s->accent = s->accent_amount; s->vcf_env_decay = s->vcf_decay_accent;
  }
}

// MIDI note -> packed 303 note (note-MIDI_NOTE_OFFSET | accent<<6 | slide<<7).
void tb303_note_on(TB303 *s, uint8_t midi_note, uint8_t velocity) {
  s->ext_control = 1;  // external control is sticky: internal sequencer stays off
  uint8_t packed = (uint8_t)((midi_note - MIDI_NOTE_OFFSET) & 0x3f);
  if (velocity > 64) packed |= 1 << 6;
  uint8_t prev = s->midi_note;
  if (prev != 0xff && s->prev_midi_note != 0xff) {
    // Stack already full (current + held previous): a third overlapping
    // note almost always means a slid-from note was never released.
    // Drop the oldest and retrigger instead of locking into legato
    // (which decays to permanent silence). Two-note slides unaffected.
    s->prev_midi_note = 0xff;
    s->midi_note = packed;
    apply_note(s, packed, 0, 1);
    return;
  }
  if (prev != 0xff) packed |= 1 << 7;
  s->prev_midi_note = prev; s->midi_note = packed;
  apply_note(s, packed, prev == 0xff ? 0 : prev, 1);  // 0 = no previous: attack
}

void tb303_note_off(TB303 *s, uint8_t midi_note) {
  uint8_t n = (uint8_t)((midi_note - MIDI_NOTE_OFFSET) & 0x3f);
  if (n == (s->midi_note & 0x3f)) { s->midi_note = s->prev_midi_note; s->prev_midi_note = 0xff; }
  if (n == (s->prev_midi_note & 0x3f)) s->prev_midi_note = 0xff;
  if (s->midi_note == 0xff && (s->current_note & 0x80) == 0) s->vca_env_state = 3;
}

void tb303_pitch_bend(TB303 *s, int16_t bend) {
  // +/-2 semitones, centred on the tuning trim. Quantised to even steps:
  // odd note-table indices hold inverse values, not increments (see above).
  s->tune = (int16_t)(TUNE_TRIM_STEPS + ((bend >> 6) & ~1));
}

// Advance the oscillator one sample and return its raw (pre-filter) value.
static inline int32_t vco_step(TB303 *s) {
  s->saw += s->freq;
  int32_t wave;
  if (s->waveform == 1) {  // square: interpolated per-note wavetable
    uint16_t p = (uint16_t)(s->saw >> 22), f = (uint16_t)(s->saw >> 6);
    wave = (sq_wavs[s->sq_note & 0x3f][p] * (int32_t)(0x10000 - f) +
            sq_wavs[s->sq_note & 0x3f][(p + 1) & 0x03ff] * (int32_t)f) >> 16;
  } else {                 // saw with band-limited edge correction
    wave = (int32_t)(s->saw >> 16);
    uint16_t phase = (uint16_t)wave + s->freq_offset;
    if (phase < s->threshold) {
      int16_t t1 = (int16_t)wave;
      wave -= saw_edge[(((t1 * (int32_t)s->freq_inv) >> 13) + 1024) & 0x7ff];
    }
    wave = 0x7fff - wave;
  }
  return wave;
}

int32_t tb303_render_vco(TB303 *s) { return vco_step(s); }

// Full per-sample body. Returns the pre-quantisation output (float,
// roughly +/-32768 scale); dac_out optionally receives the legacy
// 12-bit DAC value ((expr + 0x8000) >> 4) so one call can feed both
// a DAC and a high-resolution I2S path without advancing state twice.
static float render_body(TB303 *s, int16_t *dac_out) {
  int32_t wave = vco_step(s);

  int temp3 = (int)((wave >> 1) - (int32_t)s->cap_reso2);
  if (temp3 > 0xfe00) temp3 = 0xfe00; else if (temp3 < -65000) temp3 = -65000;
  uint16_t temp4 = (uint16_t)((temp3 < 0 ? -temp3 : temp3) >> 8) & 0x00ff;
  if (temp3 < 0) {
    temp3 &= 0xff;
    temp3 = -((tanh_table[temp4] * (256 - temp3) + tanh_table[temp4 + 1] * temp3) >> 8);
  } else {
    temp3 &= 0xff;
    temp3 = (tanh_table[temp4] * (256 - temp3) + tanh_table[temp4 + 1] * temp3) >> 8;
  }

  // Filter state/coefficients cached in locals: on FPU-less-pointer-chasing
  // targets (RISC-V/Xtensa at -Os) struct reloads per iteration add up.
  float cap1 = s->cap1, cap2 = s->cap2, cap3 = s->cap3, cap4 = s->cap4;
  float cap_out = s->cap_out, cap_reso = s->cap_reso, cap_reso2 = s->cap_reso2;
  const float k0 = s->k0, k1 = s->k1, k2 = s->k2, k3 = s->k3, k4 = s->k4, k5 = s->k5;
  const float reso = s->resonance, k6 = s->k6, k10 = s->k10;
  const float drive = k0 * (float)temp3;
  for (int i = 0; i < 4; i++) {
    cap1 += k1 * (drive - (cap1 + cap2));
    cap2 += k2 * cap1;
    cap3 += k3 * (cap2 - (cap3 + cap4));
    float t1 = k4 * cap3;
    cap4 += t1; t1 -= k5 * cap_out; cap_out += t1;
    float t2 = reso * t1 - k6 * (cap_reso - cap_reso2);
    cap_reso += t2; cap_reso2 += t2 - k10 * cap_reso2;
  }
  s->cap1 = cap1; s->cap2 = cap2; s->cap3 = cap3; s->cap4 = cap4;
  s->cap_out = cap_out; s->cap_reso = cap_reso; s->cap_reso2 = cap_reso2;
  float t1 = 4.0f * cap_out - s->last_average;
  s->last_average = 4.0f * cap_out;
  s->cap_vca1 += t1 - s->k7 * s->cap_vca1;
  s->cap_vca2 += t1 - s->k8 * s->cap_vca2;
  float expr = (s->vca_env + 2.855f * s->accent_vca) *
               (s->cap_vca1 + 0.42f * reso * s->cap_vca2) * 0.04f;
  int16_t output = (int16_t)(expr + 0x8000) >> 4;
  if (dac_out) *dac_out = output;

  if (++s->timer2 == 3) {
    s->timer2 = 0;
    int16_t t5 = (int16_t)(s->cutoff + 615.0f * (s->vcf_env - 0.326f) * s->env_mod + 1353.0f * s->accent_vcf);
    if (t5 < 0) t5 = 0; if (t5 > 1023) t5 = 1023; t5 = (int16_t)(t5 * 6);
    // Table gains are per-5us-step integrators: scale to the actual step.
    const float rs = s->rate_scale;
    s->k1 = cv_table[t5] * rs; s->k2 = cv_table[t5 + 1] * rs; s->k3 = cv_table[t5 + 2] * rs;
    s->k4 = cv_table[t5 + 3] * rs; s->k5 = cv_table[t5 + 4] * rs; s->k0 = cv_table[t5 + 5] * rs;
  }

  // VCF envelope + accent (attack / decay / release as in myISR)
  if (s->vcf_env_state == 1) {
    s->vcf_env += s->vcf_atk_alpha * (1.0f - s->vcf_env);
    if (s->vcf_env > 0.99998f) { s->vcf_env = 0.99998f; s->vcf_env_state = 2; s->vcf_env_timer1 = 0; }
    float t2 = s->accent * s->vcf_env - 0.362f - s->accent_cap1;
    if (t2 > 0) { s->accent_cap1 += s->k13 * t2 - s->k14 * s->accent_cap1; s->accent_vcf = s->k15 * t2 + s->accent_cap1; }
    else { s->accent_cap1 -= s->k14 * s->accent_cap1; s->accent_vcf = s->accent_cap1; }
    s->accent_cap2 += s->accent_lp_attack * (s->accent * s->vcf_env - s->accent_cap2);
    s->accent_vca = s->accent_cap2 > 0.09f ? s->accent_cap2 - 0.09f : 0;
  } else if (s->vcf_env_state == 2) {
    if (++s->vcf_env_timer1 == s->env_div) {
      s->vcf_env_timer1 = 0; s->vcf_env *= s->vcf_env_decay;
      if (s->vcf_env < 0.000015f) { s->vcf_env = 0; s->vcf_env_state = 3; }
      float t2 = s->accent * s->vcf_env - 0.362f - s->accent_cap1;
      if (t2 > 0) { s->accent_cap1 += s->k11 * t2 - s->k12 * s->accent_cap1; s->accent_vcf = s->k15 * t2 + s->accent_cap1; }
      else { s->accent_cap1 -= s->k12 * s->accent_cap1; s->accent_vcf = s->accent_cap1; }
      s->accent_cap2 += s->accent_lp_decay * (s->accent * s->vcf_env - s->accent_cap2);
      s->accent_vca = s->accent_cap2 > 0.09f ? s->accent_cap2 - 0.09f : 0;
    }
  } else if (s->vcf_env_state == 3) {
    if (++s->vcf_env_timer1 == s->env_div) {
      s->vcf_env_timer1 = 0; s->accent_cap1 -= s->k12 * s->accent_cap1; s->accent_vcf = s->accent_cap1;
      if (s->accent_vcf < 0.000015f) { s->accent_vcf = 0; s->vcf_env_state = 0; }
    }
  } else s->vcf_env = 0;

  // VCA envelope
  if (s->vca_env_state == 0) { s->vca_env = 0; s->vca_delay_cap *= s->vca_delay_decay; }
  else if (s->vca_env_state == 1) {
    s->vca_delay_cap += s->vca_charge_inc;
    if (s->vca_delay_cap > 0.4828f) {
      s->vca_delay_cap = 0.565f; s->vca_env = s->vca_env * s->vca_atk_mul + s->vca_atk_inc;
      if (s->vca_env > 0.99998f) { s->vca_env = 0.99998f; s->vca_env_state = 2; s->vca_env_timer1 = s->vca_env_timer2 = 0; s->vca_env_decay = s->vca_decay0; }
    }
  } else if (s->vca_env_state == 2) {
    if (++s->vca_env_timer1 == s->env_div) {
      s->vca_env_timer1 = 0; s->vca_env *= s->vca_env_decay;
      if (++s->vca_env_timer2 == (uint8_t)(s->env_div * 10)) { s->vca_env_timer2 = 0; s->vca_env_decay -= 0.000001f * s->rate_scale; }
      if (s->vca_env < 0.000015f) { s->vca_env = 0; s->vca_env_state = 0; }
    }
  } else if (s->vca_env_state == 3) {
    // Release decays exponentially to zero (the original code's
    // last_vca_env was never assigned, so it always targeted 0).
    s->vca_delay_cap *= s->vca_delay_decay; s->vca_env *= s->vca_release_mul;
    if (s->vca_env <= 0.000015f) { s->vca_env = 0; s->vca_env_state = 0; }
  }

  // slide
  if (s->slide_timer != 0xff && ++s->slide_timer == s->env_div) {
    s->slide_timer = 0;
    s->slide_cap += s->slide_resistor * ((float)s->current_note_value - s->slide_cap);
    uint16_t tn = (uint16_t)((uint16_t)s->slide_cap + s->tune);
    if (tn > 4094) tn = 4094;
    s->sq_note = (uint16_t)((tn >> 6) & 0x003f);
    uint16_t interim = note_table[tn];
    s->freq = (uint32_t)((float)(interim << 10) * s->rate_scale); s->freq_offset = (uint16_t)(interim >> 3);
    s->threshold = (uint16_t)(interim >> 2); s->freq_inv = note_table[tn + 1];
  }

  // internal sequencer: plays only until external control (notes or
  // clock) takes over; from then on the engine is externally owned.
  if (!s->ext_clock && !s->ext_control && ++s->tempo_counter >= s->tempo_base) {
    s->tempo_counter = 0;
    if (++s->step_counter >= 12) {
      s->step_counter = 0;
      uint8_t prev = s->current_note;
      if (++s->current_step >= s->sequence_length) s->current_step = 0;
      apply_note(s, s->sequence[s->current_step], prev, 0);
    } else if (s->step_counter == 7 && (s->current_note & 0x80) == 0) {
      s->vca_env_state = 3;
    }
  }
  return expr;
}

int16_t tb303_render(TB303 *s) {
  int16_t dac;
  render_body(s, &dac);
  return dac;
}

float tb303_render_float(TB303 *s) {
  float f = render_body(s, NULL) * (1.0f / 32768.0f);
  return f > 1.0f ? 1.0f : f < -1.0f ? -1.0f : f;
}

void tb303_clock(TB303 *s, uint8_t msg) {
  if (msg == 0xFA) { s->ext_clock = 1; s->ext_control = 1; s->pulse_counter = 0; s->current_step = 0; }
  else if (msg == 0xFC) { s->ext_clock = 0; s->pulse_counter = 0; s->vca_env_state = 3; }
  else if (msg == 0xF8 && s->ext_clock) {
    if (++s->pulse_counter >= s->pulses_per_step) {
      s->pulse_counter = 0;
      uint8_t prev = s->current_note;
      if (++s->current_step >= s->sequence_length) s->current_step = 0;
      apply_note(s, s->sequence[s->current_step], prev, 0);
    } else if (s->pulse_counter == s->pulses_per_note &&
               (s->current_note & 0x80) == 0) {
      s->vca_env_state = 3;
    }
  }
}
