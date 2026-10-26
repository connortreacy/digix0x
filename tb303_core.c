// TB-303 DSP core implementation. See tb303_core.h.
// Filter/envelope math transcribed verbatim from digix0x.ino (main bec716c);
// MIDI gate/accent/slide semantics from midi-current (d23eb96).
#include "tb303_core.h"

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
#define Ts 0.000005f

static const uint8_t default_sequence[16] = {
#include "everybody303.h"
};

void tb303_init(TB303 *s) {
  memset(s, 0, sizeof(*s));
  s->k0 = 1.47f; s->k1 = 0.2876302f; s->k2 = 0.0797963f;
  s->k3 = 0.0654076f; s->k4 = 0.0072146f; s->k5 = 0.0013524f;
  s->k6 = Ts * 6.283f * HPF_Q * HPF_W;
  s->k10 = Ts * 6.283f * HPF_W / HPF_Q;
  s->k7 = 0.00785f; s->k8 = 0.0204f;
  s->vca_env_decay = 0.99991f; s->vcf_env_decay = 0.9972f;
  s->k11 = 0.0018f; s->k12 = 0.00143f;
  s->k13 = s->k11 / 10.0f; s->k14 = s->k12 / 10.0f;
  s->slide_resistor = 0.00909f; s->slide_timer = 0xff;
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
  s->cutoff = 0.332f * cutoff + 328.0f;
  s->env_mod = 0.16667f + 0.0008138f * env_mod;
  s->pot_decay = decay; s->waveform = waveform;
}

static void apply_note(TB303 *s, uint8_t packed, uint8_t prev_packed) {
  s->current_note = packed;
  if ((packed & 0x3f) != 0) s->current_note_value = (int16_t)((packed & 0x3f) << 6);
  if ((prev_packed & 0x80) == 0) {           // attack unless previous slid
    s->vca_env_state = 1; s->vcf_env_state = 1; s->slide_timer = 0xff;
    uint16_t tn = (uint16_t)(s->current_note_value + s->tune);
    if (tn > 4094) tn = 4094;
    s->sq_note = (uint16_t)((tn >> 6) & 0x003f);
    uint16_t interim = note_table[tn];
    s->freq = (uint32_t)interim << 10;
    s->freq_offset = (uint16_t)(interim >> 3);
    s->threshold = (uint16_t)(interim >> 2);
    s->freq_inv = note_table[tn + 1];
    s->slide_cap = (float)s->current_note_value;
  } else {
    s->slide_timer = 0;                       // legato: slide, no retrigger
  }
  if ((packed & 0x40) == 0) {
    s->accent = 0; s->vcf_env_decay = decay_table[s->pot_decay & 0x3ff];
  } else {
    s->accent = s->accent_amount; s->vcf_env_decay = 0.9972f;
  }
}

// MIDI note -> packed 303 note (note-23 | accent<<6 | slide<<7).
void tb303_note_on(TB303 *s, uint8_t midi_note, uint8_t velocity) {
  uint8_t packed = (uint8_t)((midi_note - 23) & 0x3f);
  if (velocity > 64) packed |= 1 << 6;
  uint8_t prev = s->midi_note;
  if (prev != 0) packed |= 1 << 7;
  s->prev_midi_note = prev; s->midi_note = packed;
  apply_note(s, packed, prev);
}

void tb303_note_off(TB303 *s, uint8_t midi_note) {
  uint8_t n = (uint8_t)((midi_note - 23) & 0x3f);
  if (n == (s->midi_note & 0x3f)) s->midi_note = s->prev_midi_note, s->prev_midi_note = 0;
  if (n == (s->prev_midi_note & 0x3f)) s->prev_midi_note = 0;
  if (s->midi_note == 0 && (s->current_note & 0x80) == 0) s->vca_env_state = 3;
}

void tb303_pitch_bend(TB303 *s, int16_t bend) {
  s->tune = (int16_t)(bend >> 7);  // +/-64 steps of the 64-steps/semitone table
}

int16_t tb303_render(TB303 *s) {
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

  for (int i = 0; i < 4; i++) {
    s->cap1 += s->k1 * (s->k0 * temp3 - (s->cap1 + s->cap2));
    s->cap2 += s->k2 * s->cap1;
    s->cap3 += s->k3 * (s->cap2 - (s->cap3 + s->cap4));
    float t1 = s->k4 * s->cap3;
    s->cap4 += t1; t1 -= s->k5 * s->cap_out; s->cap_out += t1;
    float t2 = s->resonance * t1 - s->k6 * (s->cap_reso - s->cap_reso2);
    s->cap_reso += t2; s->cap_reso2 += t2 - s->k10 * s->cap_reso2;
  }
  s->average = 4.0f * s->cap_out;
  float t1 = s->average - s->last_average;
  s->last_average = s->average; s->average = 0;
  s->cap_vca1 += t1 - s->k7 * s->cap_vca1;
  s->cap_vca2 += t1 - s->k8 * s->cap_vca2;
  int16_t output = (int16_t)(((s->vca_env + 2.855f * s->accent_vca) *
                   (s->cap_vca1 + 0.42f * s->resonance * s->cap_vca2) * 0.04f) + 0x8000) >> 4;

  if (++s->timer2 == 3) {
    s->timer2 = 0;
    int16_t t5 = (int16_t)(s->cutoff + 615.0f * (s->vcf_env - 0.326f) * s->env_mod + 1353.0f * s->accent_vcf);
    if (t5 < 0) t5 = 0; if (t5 > 1023) t5 = 1023; t5 = (int16_t)(t5 * 6);
    s->k1 = cv_table[t5]; s->k2 = cv_table[t5 + 1]; s->k3 = cv_table[t5 + 2];
    s->k4 = cv_table[t5 + 3]; s->k5 = cv_table[t5 + 4]; s->k0 = cv_table[t5 + 5];
  }

  // VCF envelope + accent (attack / decay / release as in myISR)
  if (s->vcf_env_state == 1) {
    s->vcf_env += 0.15f * (1.0f - s->vcf_env);
    if (s->vcf_env > 0.99998f) { s->vcf_env = 0.99998f; s->vcf_env_state = 2; s->vcf_env_timer1 = 0; }
    float t2 = s->accent * s->vcf_env - 0.362f - s->accent_cap1;
    if (t2 > 0) { s->accent_cap1 += s->k13 * t2 - s->k14 * s->accent_cap1; s->accent_vcf = s->k15 * t2 + s->accent_cap1; }
    else { s->accent_cap1 -= s->k14 * s->accent_cap1; s->accent_vcf = s->accent_cap1; }
    s->accent_cap2 += 0.0404f * (s->accent * s->vcf_env - s->accent_cap2);
    s->accent_vca = s->accent_cap2 > 0.09f ? s->accent_cap2 - 0.09f : 0;
  } else if (s->vcf_env_state == 2) {
    if (++s->vcf_env_timer1 == 10) {
      s->vcf_env_timer1 = 0; s->vcf_env *= s->vcf_env_decay;
      if (s->vcf_env < 0.000015f) { s->vcf_env = 0; s->vcf_env_state = 3; }
      float t2 = s->accent * s->vcf_env - 0.362f - s->accent_cap1;
      if (t2 > 0) { s->accent_cap1 += s->k11 * t2 - s->k12 * s->accent_cap1; s->accent_vcf = s->k15 * t2 + s->accent_cap1; }
      else { s->accent_cap1 -= s->k12 * s->accent_cap1; s->accent_vcf = s->accent_cap1; }
      s->accent_cap2 += 0.404f * (s->accent * s->vcf_env - s->accent_cap2);
      s->accent_vca = s->accent_cap2 > 0.09f ? s->accent_cap2 - 0.09f : 0;
    }
  } else if (s->vcf_env_state == 3) {
    if (++s->vcf_env_timer1 == 10) {
      s->vcf_env_timer1 = 0; s->accent_cap1 -= s->k12 * s->accent_cap1; s->accent_vcf = s->accent_cap1;
      if (s->accent_vcf < 0.000015f) { s->accent_vcf = 0; s->vcf_env_state = 0; }
    }
  } else s->vcf_env = 0;

  // VCA envelope
  if (s->vca_env_state == 0) { s->vca_env = 0; s->vca_delay_cap *= 0.9998f; }
  else if (s->vca_env_state == 1) {
    s->vca_delay_cap += 0.002f;
    if (s->vca_delay_cap > 0.4828f) {
      s->vca_delay_cap = 0.565f; s->vca_env = s->vca_env * 1.076f + 0.0076f;
      if (s->vca_env > 0.99998f) { s->vca_env = 0.99998f; s->vca_env_state = 2; s->vca_env_timer1 = s->vca_env_timer2 = 0; s->vca_env_decay = 0.99991f; }
    }
  } else if (s->vca_env_state == 2) {
    if (++s->vca_env_timer1 == 10) {
      s->vca_env_timer1 = 0; s->vca_env *= s->vca_env_decay;
      if (++s->vca_env_timer2 == 100) { s->vca_env_timer2 = 0; s->vca_env_decay -= 0.000001f; }
      if (s->vca_env < 0.000015f) { s->vca_env = 0; s->vca_env_state = 0; }
    }
  } else if (s->vca_env_state == 3) {
    s->vca_delay_cap *= 0.9998f; s->vca_env += 0.004f * (s->last_vca_env - s->vca_env);
    if (s->vca_env <= 0.000015f) { s->vca_env = 0; s->vca_env_state = 0; }
  }
  s->last_vca_env = s->vca_env;

  // slide
  if (s->slide_timer != 0xff && ++s->slide_timer == 10) {
    s->slide_timer = 0;
    s->slide_cap += s->slide_resistor * ((float)s->current_note_value - s->slide_cap);
    uint16_t tn = (uint16_t)((uint16_t)s->slide_cap + s->tune);
    if (tn > 4094) tn = 4094;
    s->sq_note = (uint16_t)((tn >> 6) & 0x003f);
    uint16_t interim = note_table[tn];
    s->freq = (uint32_t)interim << 10; s->freq_offset = (uint16_t)(interim >> 3);
    s->threshold = (uint16_t)(interim >> 2); s->freq_inv = note_table[tn + 1];
  }

  // internal sequencer (free-running when no external MIDI clock)
  if (!s->ext_clock && ++s->tempo_counter >= 500) {
    s->tempo_counter = 0;
    if (++s->step_counter >= 12) {
      s->step_counter = 0;
      uint8_t prev = s->current_note;
      if (++s->current_step >= s->sequence_length) s->current_step = 0;
      apply_note(s, s->sequence[s->current_step], prev);
    } else if (s->step_counter == 7 && (s->current_note & 0x80) == 0) {
      s->vca_env_state = 3;
    }
  }
  return output;
}

void tb303_clock(TB303 *s, uint8_t msg) {
  if (msg == 0xFA) { s->ext_clock = 1; s->pulse_counter = 0; s->current_step = 0; }
  else if (msg == 0xFC) { s->ext_clock = 0; s->pulse_counter = 0; s->vca_env_state = 3; }
  else if (msg == 0xF8 && s->ext_clock) {
    if (++s->pulse_counter >= s->pulses_per_step) {
      s->pulse_counter = 0;
      uint8_t prev = s->current_note;
      if (++s->current_step >= s->sequence_length) s->current_step = 0;
      apply_note(s, s->sequence[s->current_step], prev);
    } else if (s->pulse_counter == s->pulses_per_note &&
               (s->current_note & 0x80) == 0) {
      s->vca_env_state = 3;
    }
  }
}
