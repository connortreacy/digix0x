// Regression test for MIDI tuning calibration (see validate_vco.py):
// MIDI note numbers must produce true pitch (A440) within +/-20 cents,
// and pitch-bend centre must not detune. Fails on the original -23 offset.
#include <stdio.h>
#include <math.h>
#include "../tb303_core.h"

static double measure_hz(uint8_t midi, int bend) {
  TB303 s; tb303_init(&s);
  tb303_set_controls(&s, 512, 512, 512, 512, 512, 0);
  tb303_note_on(&s, midi, 100);
  if (bend) tb303_pitch_bend(&s, 0);
  const int settle = 5000, n = CONFIG_SAMPLE_RATE;
  int32_t prev = 0; long cross = 0;
  for (int i = 0; i < settle + n; i++) {
    int32_t v = tb303_render_vco(&s);
    if (i >= settle && prev < 0 && v >= 0) cross++;
    prev = v;
  }
  return (double)cross;  // rising crossings over exactly 1 s
}

int main(void) {
  const struct { uint8_t midi; double hz; } cases[] = {{36, 65.41}, {57, 220.0}, {69, 440.0}};
  int fails = 0;
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    double f = measure_hz(cases[i].midi, 0);
    double cents = 1200.0 * log2(f / cases[i].hz);
    printf("MIDI %d: %.2f Hz (expect %.2f, %+.1f cents)\n", cases[i].midi, f, cases[i].hz, cents);
    if (fabs(cents) > 20.0) fails++;
  }
  double f0 = measure_hz(69, 0), fb = measure_hz(69, 1);
  printf("bend-centre drift: %+.1f cents\n", 1200.0 * log2(fb / f0));
  if (fabs(1200.0 * log2(fb / f0)) > 5.0) fails++;
  printf(fails ? "FAIL\n" : "OK\n");
  return fails != 0;
}
