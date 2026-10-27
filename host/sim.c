// digix0x simulator for macOS: realtime CoreAudio playback of tb303_core,
// computer-keyboard piano, knob keys, and CoreMIDI input. Core renders at
// 50 kHz and is linearly resampled to the device rate in the callback.
//   make sim && ./sim           play (q or Ctrl-C quits)
//   ./sim --selftest            offline 2 s render -> sim_test.wav, exits
// Keys: A W S E D F T G Y H U J K O L P ; '  chromatic piano from C
//       UPPERCASE = accented note, Z/X octave, SPACE note off,
//       1-5 select knob, -/= adjust, V waveform, ENTER sequencer on/off.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>
#include <AudioToolbox/AudioToolbox.h>
#include <CoreMIDI/CoreMIDI.h>
#include "../tb303_core.h"

static TB303 g_synth;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static uint16_t g_pot[5] = {512, 512, 512, 512, 512};  // accent reso cutoff decay envmod
static uint8_t g_wave;
static int g_octave = 3, g_selected;
static volatile int g_held_note = -1;

// ---- resampler state (audio thread only) ----
static float rs_prev, rs_cur, rs_frac;
static float next_core_sample(void) {
  // Full-resolution float render (the 12-bit DAC path floors at ~-74 dBFS,
  // audible as hiss once gained up). x3 makeup: core peaks near 0.28 FS.
  float f = tb303_render_float(&g_synth) * 3.0f;
  return f > 1.0f ? 1.0f : f < -1.0f ? -1.0f : f;
}
static OSStatus render_cb(void *ref, AudioUnitRenderActionFlags *flags,
                          const AudioTimeStamp *ts, UInt32 bus, UInt32 frames,
                          AudioBufferList *io) {
  (void)ref; (void)flags; (void)ts; (void)bus;
  float *out = (float *)io->mBuffers[0].mData;
  UInt32 channels = io->mBuffers[0].mNumberChannels;
  double rate = *(double *)ref;
  double step = CONFIG_SAMPLE_RATE / rate;
  pthread_mutex_lock(&g_lock);
  for (UInt32 i = 0; i < frames; i++) {
    rs_frac += (float)step;
    while (rs_frac >= 1.0f) { rs_frac -= 1.0f; rs_prev = rs_cur; rs_cur = next_core_sample(); }
    float s = rs_prev + (rs_cur - rs_prev) * rs_frac;
    for (UInt32 c = 0; c < channels; c++) out[i * channels + c] = s;
  }
  pthread_mutex_unlock(&g_lock);
  return noErr;
}

static void apply_controls(void) {
  pthread_mutex_lock(&g_lock);
  tb303_set_controls(&g_synth, g_pot[0], g_pot[1], g_pot[2], g_pot[3], g_pot[4], g_wave);
  pthread_mutex_unlock(&g_lock);
}
static void note_on(int midi, int accent) {
  pthread_mutex_lock(&g_lock);
  tb303_note_on(&g_synth, (uint8_t)midi, accent ? 110 : 50);  // core: vel>64 = accent
  pthread_mutex_unlock(&g_lock);
  g_held_note = midi;
}
static void note_off(int midi) {
  pthread_mutex_lock(&g_lock);
  tb303_note_off(&g_synth, (uint8_t)midi);
  pthread_mutex_unlock(&g_lock);
  if (g_held_note == midi) g_held_note = -1;
}

// ---- CoreMIDI: feed every source into the core ----
static void midi_read(const MIDIPacketList *list, void *ref, void *src) {
  (void)ref; (void)src;
  const MIDIPacket *pkt = &list->packet[0];
  for (UInt32 i = 0; i < list->numPackets; i++) {
    for (int b = 0; b + 2 < pkt->length; ) {
      uint8_t st = pkt->data[b];
      if (st == 0xF8 || st == 0xFA || st == 0xFC) {
        pthread_mutex_lock(&g_lock); tb303_clock(&g_synth, st); pthread_mutex_unlock(&g_lock);
        b++; continue;
      }
      uint8_t cmd = st & 0xF0;
      if (cmd == 0x90 || cmd == 0x80 || cmd == 0xB0 || cmd == 0xE0) {
        uint8_t d1 = pkt->data[b + 1], d2 = pkt->data[b + 2];
        pthread_mutex_lock(&g_lock);
        if (cmd == 0x90 && d2 > 0) tb303_note_on(&g_synth, d1, d2);
        else if (cmd == 0x80 || cmd == 0x90) tb303_note_off(&g_synth, d1);
        else if (cmd == 0xE0) tb303_pitch_bend(&g_synth, (int16_t)(((d2 << 7) | d1) - 8192));
        pthread_mutex_unlock(&g_lock);
        b += 3;
      } else b++;
    }
    pkt = MIDIPacketNext(pkt);
  }
}
static void setup_midi(void) {
  MIDIClientRef client; MIDIPortRef port;
  if (MIDIClientCreate(CFSTR("digix0x-sim"), NULL, NULL, &client) != noErr) return;
  if (MIDIInputPortCreate(client, CFSTR("in"), midi_read, NULL, &port) != noErr) return;
  ItemCount n = MIDIGetNumberOfSources();
  for (ItemCount i = 0; i < n; i++) MIDIPortConnectSource(port, MIDIGetSource(i), NULL);
  if (n) printf("CoreMIDI: listening on %d source(s)\r\n", (int)n);
}

// ---- very basic 16-step sequencer (drives the core like a MIDI source) ----
static uint8_t seq_note[16] = {36,0,36,39, 36,0,41,36, 36,0,36,43, 41,39,36,0};
static uint8_t seq_accent[16] = {1,0,0,1, 0,0,1,0, 1,0,0,1, 0,1,0,0};
static uint8_t seq_slide[16]  = {0,0,0,0, 0,0,1,0, 0,0,0,0, 1,0,0,0};
static int seq_playing, seq_step, seq_cursor, edit_mode;
static double seq_bpm = 140.0, seq_next, seq_gate_off;
static double now_s(void) {
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static void seq_start(void) { seq_playing = 1; seq_step = 0; seq_next = now_s(); seq_gate_off = 0; }
static void seq_stop(void) {
  seq_playing = 0;
  if (g_held_note >= 0) note_off(g_held_note);
}
static void seq_tick(void) {
  if (!seq_playing) return;
  double t = now_s(), step_dur = 60.0 / seq_bpm / 4.0;
  if (seq_gate_off > 0 && t >= seq_gate_off) {
    seq_gate_off = 0;
    int nxt = (seq_step + 1) & 15;
    if (!seq_note[nxt] || !seq_slide[nxt]) { if (g_held_note >= 0) note_off(g_held_note); }
  }
  if (t < seq_next) return;
  seq_next = t + step_dur;
  int s = seq_step; seq_step = (seq_step + 1) & 15;
  if (!seq_note[s]) { seq_gate_off = t; return; }
  if (!(seq_slide[s] && g_held_note >= 0) && g_held_note >= 0) note_off(g_held_note);
  note_on(seq_note[s], seq_accent[s]);
  seq_gate_off = seq_slide[s] ? 0 : t + step_dur * 0.65;
}

// ---- UI ----
static const char *NOTE_NAMES[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
static const char *KNOB_NAMES[5] = {"ACCENT","RESO","CUTOFF","DECAY","ENVMOD"};
static void print_ui(void) {
  printf("\r\033[Kdigix0x sim | %s | bpm %.0f | wave %s | oct %d | held %d | ",
         edit_mode ? "EDIT" : "PLAY", seq_bpm, g_wave ? "SQUARE" : "SAW", g_octave, g_held_note);
  for (int i = 0; i < 5; i++) printf("%s %s:%3d ", i == g_selected ? ">" : " ", KNOB_NAMES[i], g_pot[i] * 100 / 1023);
  printf("\r\n\033[K");
  for (int s = 0; s < 16; s++) {
    char buf[8] = "...";
    if (seq_note[s]) snprintf(buf, sizeof buf, "%s%d", NOTE_NAMES[seq_note[s] % 12], seq_note[s] / 12 - 1);
    printf("%s%-4s%s", s == seq_cursor ? "[" : " ", buf, s == seq_cursor ? "]" : " ");
  }
  printf("\r\n\033[K");
  for (int s = 0; s < 16; s++)
    printf(" %-4c ", seq_playing && s == ((seq_step + 15) & 15) ? '^' :
           seq_accent[s] && seq_slide[s] ? '~' : seq_accent[s] ? '!' : seq_slide[s] ? '~' : '.');
  printf("\r\n\033[3A");
  fflush(stdout);
}

static void selftest(void) {
  tb303_init(&g_synth);
  tb303_clock(&g_synth, 0xFA);  // sim is clock master
  apply_controls();
  seq_playing = 1;
  const int rate = 48000, secs = 4, n = rate * secs;
  int16_t *buf = malloc((size_t)n * 2);
  double step = (double)CONFIG_SAMPLE_RATE / rate, acc = 0; int peak = 0;
  // drive the sequencer in simulated time while resampling like the callback
  double sim_t = 0, dt = 1.0 / rate;
  seq_next = 0;
  for (int i = 0; i < n; i++) {
    // inline seq_tick against simulated clock
    if (seq_playing) {
      double sd = 60.0 / seq_bpm / 4.0;
      if (seq_gate_off > 0 && sim_t >= seq_gate_off) { seq_gate_off = 0; if (g_held_note >= 0) note_off(g_held_note); }
      if (sim_t >= seq_next) {
        seq_next = sim_t + sd; int s = seq_step; seq_step = (seq_step + 1) & 15;
        if (seq_note[s]) { if (!(seq_slide[s] && g_held_note >= 0) && g_held_note >= 0) note_off(g_held_note);
          note_on(seq_note[s], seq_accent[s]); seq_gate_off = seq_slide[s] ? 0 : sim_t + sd * 0.65; }
      }
    }
    rs_frac += (float)step;
    while (rs_frac >= 1.0f) { rs_frac -= 1.0f; rs_prev = rs_cur; rs_cur = next_core_sample(); }
    float f = rs_prev + (rs_cur - rs_prev) * rs_frac;
    int v = (int)(f * 32767.0f); buf[i] = (int16_t)v;
    acc += (double)v * v; if (abs(v) > peak) peak = abs(v);
    sim_t += dt;
  }
  FILE *fp = fopen("sim_test.wav", "wb");
  uint32_t data = (uint32_t)n * 2, riff = 36 + data, sub = 16, br = (uint32_t)rate * 2;
  uint16_t fmt = 1, ch = 1, ba = 2, bits = 16;
  fwrite("RIFF", 1, 4, fp); fwrite(&riff, 4, 1, fp); fwrite("WAVEfmt ", 1, 8, fp);
  fwrite(&sub, 4, 1, fp); fwrite(&fmt, 2, 1, fp); fwrite(&ch, 2, 1, fp);
  fwrite(&rate, 4, 1, fp); fwrite(&br, 4, 1, fp); fwrite(&ba, 2, 1, fp); fwrite(&bits, 2, 1, fp);
  fwrite("data", 1, 4, fp); fwrite(&data, 4, 1, fp); fwrite(buf, 2, (size_t)n, fp); fclose(fp);
  printf("sim_test.wav rms=%.1f peak=%d -> %s\n", sqrt(acc / n), peak,
         (peak > 1000 && sqrt(acc / n) > 100) ? "OK" : "FAIL");
  exit((peak > 1000 && sqrt(acc / n) > 100) ? 0 : 1);
}

static const char PIANO[] = "awsedftgyhujkolp;'";
static int piano_note(int c) {
  const char *p = strchr(PIANO, c);
  return p ? (int)(p - PIANO) : -1;
}

int main(int argc, char **argv) {
  tb303_init(&g_synth);
  tb303_clock(&g_synth, 0xFA);  // sim is clock master: silence core's internal sequencer
  if (argc > 1 && strcmp(argv[1], "--selftest") == 0) selftest();
  apply_controls();
  setup_midi();

  // AudioUnit default output at the device's native rate.
  static double out_rate = 48000;
  AudioComponent comp = AudioComponentFindNext(NULL, &(AudioComponentDescription){
    .componentType = kAudioUnitType_Output, .componentSubType = kAudioUnitSubType_DefaultOutput,
    .componentManufacturer = kAudioUnitManufacturer_Apple});
  AudioUnit unit; AudioComponentInstanceNew(comp, &unit);
  AudioStreamBasicDescription fmt;
  UInt32 size = sizeof fmt;
  if (AudioUnitGetProperty(unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 0, &fmt, &size) == noErr && fmt.mSampleRate > 0)
    out_rate = fmt.mSampleRate;
  fmt.mFormatID = kAudioFormatLinearPCM;
  fmt.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked | kAudioFormatFlagIsNonInterleaved;
  fmt.mFramesPerPacket = 1; fmt.mChannelsPerFrame = 1; fmt.mBitsPerChannel = 32;
  fmt.mBytesPerFrame = 4; fmt.mBytesPerPacket = 4;
  AudioUnitSetProperty(unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &fmt, sizeof fmt);
  AURenderCallbackStruct cb = {.inputProc = render_cb, .inputProcRefCon = &out_rate};
  AudioUnitSetProperty(unit, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &cb, sizeof cb);
  AudioUnitInitialize(unit); AudioOutputUnitStart(unit);
  printf("digix0x sim: device %.0f Hz, core %d Hz. TAB=edit, ENTER=seq, SPACE=note off, Q=quit\r\n", out_rate, CONFIG_SAMPLE_RATE);

  struct termios oldt, raw;
  tcgetattr(0, &oldt); raw = oldt; raw.c_lflag &= ~(ECHO | ICANON);
  tcsetattr(0, TCSANOW, &raw);
  print_ui();
  for (;;) {
    seq_tick();
    fd_set fds; FD_ZERO(&fds); FD_SET(0, &fds);
    struct timeval tv = {0, 2000};
    if (select(1, &fds, NULL, NULL, &tv) <= 0) continue;
    int c = getchar();
    if (c == 'q' || c == 3) break;
    if (c == '\t') { edit_mode = !edit_mode; print_ui(); continue; }
    if (c == '\r' || c == '\n') { seq_playing ? seq_stop() : seq_start(); print_ui(); continue; }
    if (edit_mode) {
      if (c == 27) { // arrow keys: ESC [ A/B/C/D
        getchar(); int a = getchar();
        if (a == 'C') seq_cursor = (seq_cursor + 1) & 15;
        else if (a == 'D') seq_cursor = (seq_cursor + 15) & 15;
        else if (a == 'A' && seq_note[seq_cursor] < 84) seq_note[seq_cursor]++;
        else if (a == 'B' && seq_note[seq_cursor] > 12) seq_note[seq_cursor]--;
      }
      else if (c == 'a') seq_accent[seq_cursor] ^= 1;
      else if (c == 'l') seq_slide[seq_cursor] ^= 1;
      else if (c == 'r') seq_note[seq_cursor] = seq_note[seq_cursor] ? 0 : 36;
      else if (c == '+' || c == '=') seq_bpm += 2;
      else if (c == '-') seq_bpm -= 2;
      print_ui(); continue;
    }
    if (c == ' ') { if (g_held_note >= 0) note_off(g_held_note); print_ui(); continue; }
    if (c >= '1' && c <= '5') { g_selected = c - '1'; print_ui(); continue; }
    if (c == 'v' || c == 'V') { g_wave ^= 1; apply_controls(); print_ui(); continue; }
    if (c == 'z') { if (g_octave > 0) g_octave--; print_ui(); continue; }
    if (c == 'x') { if (g_octave < 7) g_octave++; print_ui(); continue; }
    if (c == '-' || c == '=') {
      int d = (c == '=') ? 32 : -32;
      int v = (int)g_pot[g_selected] + d;
      g_pot[g_selected] = (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : v);
      apply_controls(); print_ui(); continue;
    }
    int semi = piano_note(c >= 'A' && c <= 'Z' ? c + 32 : c);
    if (semi >= 0) {
      if (g_held_note >= 0) note_off(g_held_note);
      note_on(g_octave * 12 + semi, c >= 'A' && c <= 'Z');
      print_ui();
    }
  }
  seq_stop();
  AudioOutputUnitStop(unit); AudioUnitUninitialize(unit);
  tcsetattr(0, TCSANOW, &oldt);
  printf("\r\nbye\r\n");
  return 0;
}
