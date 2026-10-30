#pragma once
// Pot/CC takeover with latching ("pickup"). A CC message takes control
// of its parameter immediately; the physical pot stays ignored until
// its position catches the current value (crosses it or enters the
// catch window), at which point the pot smoothly resumes control.
// This avoids both jumps when remote control starts and dead pots
// after it ends. Used by the SAMD loop and the ESP32 control scan.
#include <stdint.h>
#include <stdlib.h>

#define LATCH_CATCH_WINDOW 24  // of 1023 (~2.4%)

typedef struct {
  uint16_t value;     // current effective value 0..1023
  uint16_t cc;        // last CC value 0..1023
  uint16_t prev_pot;  // last seen pot position (crossing detection)
  uint8_t from_cc;    // 1 while CC owns this parameter
} ControlLatch;

static inline void latch_cc(ControlLatch *l, uint16_t cc_value) {
  l->cc = cc_value;
  l->value = cc_value;
  l->from_cc = 1;
}

// Call every control scan with the raw pot reading; returns effective value.
static inline uint16_t latch_pot(ControlLatch *l, uint16_t pot) {
  if (l->from_cc) {
    int eff = (int)l->value;
    int crossed = (l->prev_pot < eff && pot >= eff) || (l->prev_pot > eff && pot <= eff);
    if (crossed || abs((int)pot - eff) <= LATCH_CATCH_WINDOW) {
      l->from_cc = 0;
      l->value = pot;
    } else {
      l->value = l->cc;
    }
  } else {
    l->value = pot;
  }
  l->prev_pot = pot;
  return l->value;
}
