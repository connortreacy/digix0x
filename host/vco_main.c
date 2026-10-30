// Renders the raw VCO (pre-filter) for every packed note 0..63, saw+square,
// as 1 s 16-bit WAVs in vco_out/. Pairing with 303wavs/sawNN.wav etc. is
// resolved by measured pitch in validate_vco.py (name offset is empirical).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "../tb303_core.h"

static void write_wav(const char *path, const int16_t *pcm, int n, int rate) {
  FILE *f = fopen(path, "wb");
  if (!f) { perror(path); exit(1); }
  uint32_t data = (uint32_t)n * 2, riff = 36 + data, sub = 16, br = (uint32_t)rate * 2;
  uint16_t fmt = 1, ch = 1, ba = 2, bits = 16;
  fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
  fwrite(&sub, 4, 1, f); fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f);
  fwrite(&rate, 4, 1, f); fwrite(&br, 4, 1, f); fwrite(&ba, 2, 1, f); fwrite(&bits, 2, 1, f);
  fwrite("data", 1, 4, f); fwrite(&data, 4, 1, f); fwrite(pcm, 2, (size_t)n, f); fclose(f);
}

int main(void) {
  const int rate = CONFIG_SAMPLE_RATE, n = rate;
  int16_t *buf = malloc((size_t)n * 2);
  mkdir("vco_out", 0755);
  for (int wave = 0; wave < 2; wave++) {
    for (int packed = 0; packed < 64; packed++) {
      TB303 s; tb303_init(&s);
      tb303_set_controls(&s, 512, 512, 512, 512, 512, (uint8_t)wave);
      tb303_note_on(&s, (uint8_t)(packed + 23), 100);
      for (int i = 0; i < n; i++) {
        int32_t v = tb303_render_vco(&s);
        if (v > 32767) v = 32767; if (v < -32768) v = -32768;
        buf[i] = (int16_t)v;
      }
      char path[64];
      snprintf(path, sizeof(path), "vco_out/vco_%s_p%02d.wav", wave ? "sq" : "saw", packed);
      write_wav(path, buf, n, rate);
    }
  }
  printf("wrote 128 VCO renders to vco_out/\n");
  return 0;
}
