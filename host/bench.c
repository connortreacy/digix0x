#include <stdio.h>
#include <time.h>
#include "../tb303_core.h"
static double now_s(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return ts.tv_sec+ts.tv_nsec*1e-9; }
int main(void){
  TB303 s; tb303_init(&s);
  tb303_set_controls(&s,700,800,600,400,700,0);
  tb303_note_on(&s,45,110);
  const int N=2000000; volatile int32_t acc=0;
  double t0=now_s();
  for(int i=0;i<N;i++) acc+=tb303_render(&s);
  double dt=now_s()-t0;
  printf("saw render: %.1f ns/sample (%.2f Msamples/s) acc=%d\n", dt/N*1e9, N/dt/1e6, (int)acc);
  tb303_set_controls(&s,700,800,600,400,700,1); tb303_note_on(&s,45,110);
  t0=now_s();
  for(int i=0;i<N;i++) acc+=tb303_render(&s);
  dt=now_s()-t0;
  printf("square render: %.1f ns/sample (%.2f Msamples/s)\n", dt/N*1e9, N/dt/1e6);
  return 0;
}
