// Host harness: renders the TB-303 core to WAV for listening tests and
// A/B comparison against hardware-303 recordings. No Arduino required.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../tb303_core.h"

static void write_wav(const char *path, const int16_t *pcm, int n, int rate) {
  FILE *f = fopen(path, "wb");
  if (!f) { perror(path); exit(1); }
  uint32_t data = (uint32_t)n * 2;
  fwrite("RIFF", 1, 4, f);
  uint32_t riff = 36 + data; fwrite(&riff, 4, 1, f);
  fwrite("WAVEfmt ", 1, 8, f);
  uint32_t sub = 16; fwrite(&sub, 4, 1, f);
  uint16_t fmt = 1, ch = 1, bits = 16, ba = 2;
  uint32_t br = (uint32_t)rate * 2;
  fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&rate, 4, 1, f);
  fwrite(&br, 4, 1, f); fwrite(&ba, 2, 1, f); fwrite(&bits, 2, 1, f);
  fwrite("data", 1, 4, f); fwrite(&data, 4, 1, f);
  fwrite(pcm, 2, (size_t)n, f); fclose(f);
}

static double rms_peak(const int16_t *p, int n, int *peak) {
  double acc = 0; int pk = 0;
  for (int i = 0; i < n; i++) {
    acc += (double)p[i] * p[i];
    int a = p[i] < 0 ? -p[i] : p[i]; if (a > pk) pk = a;
  }
  *peak = pk; return sqrt(acc / n);
}

int main(void) {
  const int rate = CONFIG_SAMPLE_RATE, secs = 4, n = rate * secs;
  int16_t *buf = malloc((size_t)n * 2);
  TB303 s; tb303_init(&s);
  // Mid-panel defaults: cutoff/reso/envmod/decay/accent at ~50%.
  tb303_set_controls(&s, 512, 512, 512, 512, 512, 0);

  // Core output is unsigned DAC-scale (0..4095, centre 2048); centre+gain for WAV.
  for (int i = 0; i < n; i++) {
    int v = ((int)tb303_render(&s) - 2048) * 16;
    buf[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
  }
  write_wav("test_sequence.wav", buf, n, rate);
  int peak; double r = rms_peak(buf, n, &peak);
  printf("test_sequence.wav  rms=%.1f peak=%d\n", r, peak);
  if (peak < 1000 || r < 100.0) { printf("FAIL: sequence render too quiet\n"); return 1; }

  // MIDI phrase: C2 pattern with accent + legato slide (hw-303 comparison take).
  tb303_init(&s); tb303_set_controls(&s, 700, 800, 600, 400, 700, 0);
  const struct { uint8_t note, vel; int ms; } phrase[] = {
    {36, 100, 250}, {36, 40, 250}, {39, 110, 250}, {41, 40, 500},
    {36, 40, 250}, {43, 120, 250}, {41, 40, 250}, {39, 40, 250},
  };
  int idx = 0;
  for (unsigned pi = 0; pi < sizeof(phrase) / sizeof(phrase[0]); pi++) {
    tb303_note_on(&s, phrase[pi].note, phrase[pi].vel);
    int m = phrase[pi].ms * rate / 1000;
    for (int i = 0; i < m && idx < n; i++) {
      int v = ((int)tb303_render(&s) - 2048) * 16;
      buf[idx++] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
    }
    tb303_note_off(&s, phrase[pi].note);
  }
  while (idx < n) {
    int v = ((int)tb303_render(&s) - 2048) * 16;
    buf[idx++] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
  }
  write_wav("test_midi.wav", buf, n, rate);
  r = rms_peak(buf, n, &peak);
  printf("test_midi.wav      rms=%.1f peak=%d\n", r, peak);
  if (peak < 1000 || r < 100.0) { printf("FAIL: midi render too quiet\n"); return 1; }
  printf("OK: both renders non-silent; open the WAVs to listen/compare.\n");
  return 0;
}
