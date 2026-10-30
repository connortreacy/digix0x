// MIDI harness: drives a digix0x board (CoreMIDI destination matching
// HARNESS_DEST, default "Feather") with the everybody303 pattern and
// control sweeps, for record-and-analyse work. Scenarios:
//   harness pattern [loops] [bpm]     pattern via notes (accent/slide)
//   harness sweep-cutoff [loops]      pattern + CC74 ramp 0..127
//   harness sweep-reso [loops]        pattern + CC71 ramp 0..127
//   harness longnote [seconds]        one sustained note (click hunting)
//   harness stop                      send STOP + all notes off
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>

static MIDIPortRef g_port;
static MIDIEndpointRef g_dest;

static void send_msg(const uint8_t *msg, int len) {
  struct { MIDIPacketList list; uint8_t storage[256]; } buf;
  MIDIPacket *p = MIDIPacketListInit(&buf.list);
  MIDIPacketListAdd(&buf.list, sizeof buf, p, 0, len, msg);
  MIDISend(g_port, g_dest, &buf.list);
}
static void note_on(int n, int vel) { uint8_t m[3] = {0x90, (uint8_t)n, (uint8_t)vel}; send_msg(m, 3); }
static void note_off(int n) { uint8_t m[3] = {0x80, (uint8_t)n, 0}; send_msg(m, 3); }
static void cc(int num, int val) { uint8_t m[3] = {0xB0, (uint8_t)num, (uint8_t)val}; send_msg(m, 3); }
static void realtime(uint8_t b) { send_msg(&b, 1); }

static const uint8_t pattern[16] = {
#include "../everybody303.h"
};
#define MIDI_OFFSET 13  // packed note -> MIDI note (tb303_core MIDI_NOTE_OFFSET)

static void set_panel(int accent, int reso, int cutoff, int decay, int envmod) {
  cc(77, accent); cc(71, reso); cc(74, cutoff); cc(76, decay); cc(75, envmod);
}

static void run_pattern(int loops, double bpm, int sweep) {
  double step_s = 60.0 / bpm / 4.0;
  set_panel(76, 64, 64, 64, 76);  // accent, reso, cutoff, decay, envmod (~60/50%)
  int held = -1;
  for (int loop = 0; loop < loops; loop++) {
    for (int s = 0; s < 16; s++) {
      if (sweep == 74 || sweep == 71) {
        int v = (loop * 16 + s) * 127 / (loops * 16 - 1);
        cc(sweep, v);
      }
      uint8_t p = pattern[s];
      int note = (p & 0x3f), accent = (p >> 6) & 1, slide = (p >> 7) & 1;
      if (note == 0) {
        if (held >= 0 && !slide) { note_off(held); held = -1; }
      } else {
        if (held >= 0 && !slide) note_off(held);
        note_on(note + MIDI_OFFSET, accent ? 110 : 50);
        held = note + MIDI_OFFSET;
      }
      // Notes gate until the next step (303-style); rests and non-slide
      // steps note-off at the next step boundary in the code above.
      usleep((useconds_t)(step_s * 1e6));
    }
  }
  if (held >= 0) note_off(held);
}

int main(int argc, char **argv) {
  const char *want = getenv("HARNESS_DEST"); if (!want) want = "Feather";
  MIDIClientRef client; MIDIClientCreate(CFSTR("digix0x-harness"), NULL, NULL, &client);
  MIDIOutputPortCreate(client, CFSTR("out"), &g_port);
  for (ItemCount i = 0; i < MIDIGetNumberOfDestinations(); i++) {
    MIDIEndpointRef d = MIDIGetDestination(i);
    CFStringRef s = NULL; MIDIObjectGetStringProperty(d, kMIDIPropertyName, &s);
    char buf[128] = ""; if (s) CFStringGetCString(s, buf, sizeof buf, kCFStringEncodingUTF8);
    if (strstr(buf, want)) { g_dest = d; printf("destination: %s\n", buf); break; }
  }
  if (!g_dest) { printf("no destination matching '%s'\n", want); return 1; }
  const char *mode = argc > 1 ? argv[1] : "pattern";
  int loops = argc > 2 ? atoi(argv[2]) : 4;
  double bpm = argc > 3 ? atof(argv[3]) : 140.0;
  if (strcmp(mode, "pattern") == 0) run_pattern(loops, bpm, 0);
  else if (strcmp(mode, "sweep-cutoff") == 0) run_pattern(loops, bpm, 74);
  else if (strcmp(mode, "sweep-reso") == 0) run_pattern(loops, bpm, 71);
  else if (strcmp(mode, "longnote") == 0) {
    set_panel(76, 64, 90, 64, 76);
    note_on(48, 90);
    sleep((unsigned)loops);
    note_off(48);
  } else if (strcmp(mode, "stop") == 0) {
    realtime(0xFC);
    for (int n = 0; n < 128; n++) note_off(n);
  } else { printf("unknown mode %s\n", mode); return 1; }
  printf("done: %s\n", mode);
  return 0;
}
