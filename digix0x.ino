#include <Arduino.h>
#include "MIDIUSB.h"
#include "SAMD51_InterruptTimer.h"

// vco variables
float input = 0;
int32_t saw = 0;
uint16_t saw_last_value = 0;

// filter caps
float cap1 = 0; // lpf c1
float cap2 = 0; // lpf c2
float cap3 = 0; // lpf c3
float cap4 = 0; // lpf c4
float cap_out = 0; // hpf c into diffamp
float cap_reso = 0; // hpf c into reso
float cap_vca1 = 0; // hpf c into vca from diffamp
float cap_vca2 = 0; // hpf c into vca from reso
int16_t output = 0;
float cap_reso2 = 0; // second hpf into reso

// filter resistors C10, 0.00063 .021, T=5us
float k0 = 1.47;
float k1 = 0.2876302;
float k2 = 0.0797963;
float k3 = 0.0654076;
float k4 = 0.0072146;
float k5 = 0.0013524;

// filter parameters
#define HPF_Q 0.4
#define HPF_W 8.0 // in Hz
#define Ts 0.000005 // sample period in seconds
float k6 = Ts*6.283*HPF_Q*HPF_W; // hpf r into reso
float k10 = Ts*6.283*HPF_W/HPF_Q; // hpf r into reso2
float k7 = 0.00785; // hpf r into vca from diffamp
float k8 = 0.0204; // hpf r into vca from reso

// vca env variables
float vca_env = 0.0; // main envelope output
float vca_env_decay = 0.99991; // stock is 0.99991
uint8_t vca_env_timer1 = 0; // timers to set sample rates
uint8_t vca_env_timer2 = 0;
uint8_t vca_env_state = 0; // 0 = off, 1 = attack, 2 = decay, 3 = release (volatile?)
float last_vca_env = 0.0; // storage of last vca_env before release phase
float vca_delay_cap = 0.0; // timing for delay gate->vca

// vcf env variables
float vcf_env = 0.0; // main envelope out
float vcf_env_decay = 0.9972; // 0.9972 is shortest, 0.99978 is longest
uint16_t vcf_env_timer1 = 0; // timer to set sample rate
uint8_t vcf_env_state = 0; // 0 = off, 1 = attack, 2 = decay (volatile?)

// accent variables (used in vcf section)
float accent = 0; // accent knob setting: 0 -> 1
float accent_cap1 = 0; // cap for vcf accent
float accent_cap2 = 0; // cap for vca accent
float accent_vca = 0; // signal to vca
float accent_vcf = 0; // signal to vcf
float k11 = 0.0018; // R from env to accent cap: 0.002 R10, 0.00095 R0, 0.0013 R5, 0.0018 R8, 0.0011 R2
float k12 = 0.00143; // R from accent cap to ground: 0.0018 R10, 0.0011 R0, 0.0014 R5, 0.00165 R8, 0.0012 R2
float k13 = k11/10.0; // R from env to accent cap during attack: 0.0002 for R10, 
float k14 = k12/10.0; // R from accent cap to ground durring attack: 0.00018 for R10, 
float k15 = 0.0; // mix: 0 R10, 0.39 R0, 0.21 R5, 0 R8, 0.36 R2

// slide variables
float slide_cap = 0;
float slide_resistor = 0.00909; // Ts/22ms for stock - Ts = 10*20us
uint8_t slide_timer = 0xff;  // 0xff = off, other values for clocking while on

// controls
uint16_t reso = 10;
uint32_t timer = 0;
uint16_t cv = 120;
float resonance = 0;
float average = 0;
float last_average = 0;
byte timer2 = 0;
int16_t tune = 0;
float accent_pot = 0;
float env_mod_pot = 0;
float cutoff_pot = 0;
uint16_t decay_pot = 0;

// lookup tables
uint16_t const tanh_table[256] = {
  #include "303filt_tanh.h"
};
int16_t const sine_table[1024] = {
  #include "sinetable.h"
};
uint32_t const note_table[50*64] = {
  #include "midi_note_table_50x64.h"
};
float const cv_table[6*1024] = {
  #include "303filt_coeff_5us.h"
};
float decay_table[1024] = {
  #include "303_vcf_decay_table.h"
};

uint8_t sequence[16] = {
  //#include "default.h"
  //#include "everybody303.h"
  #include "riser.h"
};

// sequencer
uint32_t BPM = 130;

uint32_t tempo = 600; // step clock, T_step/(12*T_interrupt), T_step = 60s/BPM
uint32_t tempo_counter = 0;
uint8_t step_counter = 0;
uint8_t current_step = 0;
uint8_t sequence_length = 16;
uint8_t current_note = 0;
int16_t current_note_value = 0;
uint32_t freq = 100;
int max_value = 0;

float a = 2.7939677E-10;
float b = 6.07153216E-20;
float c = 5.573775612E-30;

void setup() {
//  Serial.begin(1000000);
//  delay(4000);
//  Serial.println("begin");
//  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  TC.startTimer(20, myISR); // 20 usec

  // midi
  Serial.begin(115200);
}

int i = 0;

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

// midi
//Pulse per quarter note. Each beat has 24 pulses.
//Tempo is based on software inner BPM.
int ppqn = 0;
int pulse = 0;
int step_reset = 0;
int step_progress = 0;

uint8_t MIDI_START = 0xFA;
uint8_t MIDI_CONTINUE = 0xFB;
uint8_t MIDI_STOP = 0xFC;
uint8_t MIDI_PULSE =  0xF8;

void loop() {
  // midi
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();

    //Count pulses and send note 
    if(rx.byte1 == MIDI_PULSE){
       ++ppqn;
       pulse = 1;
       /*
       if(ppqn == 6){
          // 1/16th step
          noteOn(1,48,127);
          step_progress = 1;
          MidiUSB.flush(); 
          ppqn = 0;
       };
       */
    }
    //Clock start byte
    else if(rx.byte1 == MIDI_START){
      //
      noteOn(1,60,0);
      step_progress = 1;
      MidiUSB.flush();
      ppqn = 0;
    }
    //Clock stop byte
    else if(rx.byte1 == MIDI_STOP){
      //
      noteOff(1,48,0);
      noteOff(1,60,0);
      step_reset = 1;
      step_progress = 0;
      MidiUSB.flush();
      ppqn = 0;
    };
    
  } while (rx.header != 0);

  // read control interface
  uint16_t temp7 = analogRead(A2); // reso
  resonance = 0.005078125*temp7; // reso 0.005078125 is stock
//  i++;
//  if (i = 1000) {
//    Serial.println(max_value);
//    i = 0;
//  }
  temp7 = 0x03ff - temp7;  // accent half of reso pot is reversed
  // accent variables set by resonance
  k15 = 0.000385*temp7;
  k14 = 0.000168 - 0.000000045*temp7;
  k13 = 0.000165 - 0.0000000675*temp7;
  k12 = 10.0*k14;
  k11 = 10.0*k13;
  temp7 = analogRead(A3); // cutoff
  cutoff_pot = 0.332*temp7 + 328;
  env_mod_pot = 0.16667 + 0.0008138*analogRead(A5); // env mod
//  float temp9 = 615*(vcf_env - 0.326)*(0.16667 + 0.0008138*temp8);
//  float temp9 = 615*(0 - 0.326)*(0.16667 + 0.0008138*temp8);
//  temp7 += (int16_t)temp9;
//  if (temp7 > 1023) temp7 = 1023;
//  if (temp7 < 0) temp7 = 0;
//  cv = 6*temp7;
  decay_pot = analogRead(A4); // decay
//  vcf_env_decay = decay_table[temp7];
  temp7 = analogRead(A1); // accent
  accent_pot = 0.0009775*temp7;
}

void myISR() {
  PORT->Group[0].OUTSET.reg = 1<<23;
  analogWrite(A0,output - 0x0500);
//  analogWrite(A1,((saw)>>20));
//  PORT->Group[0].OUTCLR.reg = 1<<23;


  saw -= freq;
//  input is +/-2.5V = 0.333*+/-32k  
//  int input = sine_table[(saw>>22)]>>4; // was 0.333*, and gave good resonance results
//  int16_t temp5 = saw >> 18;
//  input += (float)(temp4 - saw_last_value);
//  saw_last_value = temp4;
//  input -= k9*input;

  int temp3 = (int)((saw>>17) - cap_reso2);
//  if (temp3 > max_value) max_value =  temp3;
  if (temp3 > 0xfe00) temp3 = 0xfe00;
  else if (temp3 < -65000) temp3 = -65000;
  uint16_t temp4 = (abs(temp3) >> 8)&0x00ff;
  if (temp3 < 0) {
    temp3 &= 0x000000ff;
    temp3 = 0 - ((tanh_table[temp4]*(256-temp3) + tanh_table[temp4+1]*(temp3))>>8); 
  }
  else {
    temp3 &= 0x000000ff;
    temp3 = ((tanh_table[temp4]*(256-temp3) + tanh_table[temp4+1]*(temp3))>>8);
  }
//  float temp3 = (float)((saw>>17) - cap_reso2);
//  if (temp3 > 64300) temp3 = 64300;
//  else if (temp3 < -64300) temp3 = -64300;
//  float temp12 = temp3*temp3;
//  temp3 = 2*(temp3*(1-temp12*(a - temp12*(b - c*temp12))));

for (int i = 0; i < 4; i++) {
  // main filter section - 4x lowpass followed by 3x highpass
  cap1 += k1*(k0*temp3 - (cap1 + cap2));
  cap2 += k2*cap1;
  cap3 += k3*(cap2 - (cap3 + cap4));
  float temp1 = k4*cap3;
  cap4 += temp1;
  temp1 -= k5*cap_out;
  cap_out += temp1;
  float temp2 = resonance*temp1 - k6*(cap_reso - cap_reso2); // int is the reso amt
  cap_reso += temp2;
  cap_reso2 += temp2 - k10*cap_reso2;
//  average += cap_out;
}
average = 4*cap_out;
//  PORT->Group[0].OUTCLR.reg = 1<<23;
  float temp1 = (average - last_average);
  last_average = average;
  average = 0;
  cap_vca1 += temp1 - k7*cap_vca1;
  cap_vca2 += temp1 - k8*cap_vca2;
//  output = (int16_t)((last_average/4) + 0x8000) >> 4;
//  output = (int16_t)((vca_env*(cap_vca1 + 0.42*resonance*cap_vca2)/4)+0x8000)>>4;// 0, 0.51, 1.65, 2.2
//  output = (int16_t)(vcf_env*65536)>>4;
  output = (int16_t)(((vca_env + 2.855f*accent_vca)*(cap_vca1 + 0.42f*resonance*cap_vca2)*0.04f)+0x8000)>>4;

  // update filter parameters
  timer2++;
  if (timer2 == 3) {
    int16_t temp5 = (int16_t)(cutoff_pot + 615*(vcf_env - 0.326f)*env_mod_pot + 1353*accent_vcf);
    if (temp5 < 0) temp5 = 0;
    if (temp5 > 1023) temp5 = 1023;
    temp5 *= 6;
    k1 = cv_table[temp5];
    temp5++;
    k2 = cv_table[temp5];
    temp5++;
    k3 = cv_table[temp5];
    temp5++;
    k4 = cv_table[temp5];
    temp5++;
    k5 = cv_table[temp5];
    temp5++;
    k0 = cv_table[temp5];
    timer2 = 0;
  }

  // vcf envelope generation
  if (vcf_env_state == 0) vcf_env = 0; // off phase, do nothing
  else if (vcf_env_state == 1) { // attack phase
    vcf_env += 0.15f*(1- vcf_env); // attack time constant of 0.15
    if (vcf_env > .99998f) { // check if at max
      vcf_env = 0.99998f;
      vcf_env_state = 2;
      vcf_env_timer1 = 0;
    }
    // accent: reso->vcf
    float temp2 = accent*vcf_env - 0.362f - accent_cap1; // signal into cap
    if (temp2 > 0) {
      accent_cap1 += k13*temp2;
      accent_cap1 -= k14*accent_cap1;
      accent_vcf = k15*temp2 + accent_cap1;
    }
    else {
      accent_cap1 -= k14*accent_cap1;
      accent_vcf = accent_cap1;
    }
    // accent: vcf->vca
    accent_cap2 += 0.0404f*(accent*vcf_env - accent_cap2); // slight low pass
    if (accent_cap2 > 0.09f) accent_vca = accent_cap2 - 0.09f; // diode threshold
    else accent_vca = 0;
  }
  else if (vcf_env_state == 2) { // decay phase
    vcf_env_timer1++;
    if (vcf_env_timer1 == 10) { // 1/10 sample rate
      vcf_env_timer1 = 0;
      vcf_env *= vcf_env_decay; // decay
      if (vcf_env < 0.000015f) { // check if at minimum
        vcf_env = 0;
        vcf_env_state = 3; // release phase for attacks
      }
      // accent: reso->vcf
      float temp2 = accent*vcf_env - 0.362f - accent_cap1; // signal into cap
      if (temp2 > 0) {
        accent_cap1 += k11*temp2;
        accent_cap1 -= k12*accent_cap1;
        accent_vcf = k15*temp2 + accent_cap1;
      }
      else {
        accent_cap1 -= k12*accent_cap1;
        accent_vcf = accent_cap1;
      }
      // accent: vcf->vca
// not sure what to do here for the moment
// there is a slow decay to 0.9 that is a very slight difference from whats below
// not sure if its worth the extra effort to figure it out      
//      if (accent_cap2 > 0.2) accent_cap2 += 0.404*(vcf_env - accent_cap2);
//      else accent_cap2 += 0.004*(0.08 - accent_cap2);
      accent_cap2 += 0.404f*(accent*vcf_env - accent_cap2); // slight low pass
      if (accent_cap2 > 0.09f) accent_vca = accent_cap2 - 0.09f; // diode threshold
      else accent_vca = 0;
    }
  }
  else if (vcf_env_state == 3) { // release phase for attack, if needed
    vcf_env_timer1++;
    if (vcf_env_timer1 == 10) {
      vcf_env_timer1 = 0;
      // accent: reso->vcf
      accent_cap1 -= k12*accent_cap1;
      accent_vcf = accent_cap1;
      if (accent_vcf < 0.000015f) {
        accent_vcf = 0;
        vcf_env_state = 0;
      }
    }
  }

  // vca envelope generation
  if (vca_env_state == 0) { // off phase
    vca_env = 0; // set env to zero
    vca_delay_cap *= 0.9998f; // keep decaying delay cap
  }
  else if (vca_env_state == 1) { // attack phase
    vca_delay_cap += 0.002f; // charage decay cap
    if (vca_delay_cap > 0.4828f) { // if its hit vbe threshold, turn on attack
      vca_delay_cap = 0.565f; // reset to max vbe
      vca_env *= 1.076f; // do attack phase
      vca_env += 0.0076f;
      if (vca_env > .99998f) { // check if done attacking
        vca_env = 0.99998f; // prep for decay
        vca_env_state = 2;
        vca_env_timer1 = 0;
        vca_env_timer2 = 0;
        vca_env_decay = 0.99991f;
      }
    }
  }
  else if (vca_env_state == 2) { // decay phase
    vca_env_timer1++;
    if (vca_env_timer1 == 10) { // 1/10 sample rate for decay
      vca_env_timer1 = 0;
      vca_env_timer2++;
      vca_env *= vca_env_decay;
      if (vca_env_timer2 == 100) { // very small addition needed here
        vca_env_timer2 = 0; // so 1/1000 sample rate needed
        vca_env_decay -= 0.000001f; // limited by float32 resolution
      }
      if (vca_env < 0.000015f) {  // envelope off
        vca_env = 0;
        vca_env_state = 0;
      }
    }
  }
  else if (vca_env_state == 3) { // release phase
    vca_delay_cap *= 0.9998f; // decay delay cap
    vca_env += 0.004f*(last_vca_env - vca_env); // release is ~ a truncated expo decay
    if (vca_env <= 0.000015f) { // envelope off
      vca_env = 0;
      vca_env_state = 0;
    }
  }

  // process slides
  if (slide_timer != 0xff) { // dont bother sliding if off
    slide_timer++;
    if (slide_timer == 10) {
      slide_timer = 0;
      slide_cap += slide_resistor*(current_note_value - slide_cap);
      freq = note_table[(uint16_t)slide_cap + tune] << 10;
    }
  }

  // play pattern sequence
  //tempo_counter++;
  //if (tempo_counter >= tempo) {
  if(pulse=1) {
    pulse=0;
    //tempo_counter = 0;
    //step_counter++;
    //if (step_counter >= 12) {
    if (ppqn >= 6) {
      ppqn=0;
      step_counter = 0;
      current_step++;
      if (current_step >= sequence_length) current_step = 0;
      uint8_t temp1 = current_note;
      current_note = sequence[current_step];
      if ((current_note & 0x3f) != 0) current_note_value = (current_note & 0x3f)<<6; // if not a rest
      if ((temp1 & 0x80) == 0) { // only attack if prev note had no slide
        vca_env_state = 1;
        vcf_env_state = 1;
        slide_timer = 0xff; // turn off slides
        freq = note_table[current_note_value + tune] << 10;
        slide_cap = current_note_value; // prep in case next note needs slide
      }
      else slide_timer = 0; // enable slides
      if ((current_note & 0x40) == 0) {  // check if not accents
        accent = 0; // turn off accents
        vcf_env_decay = decay_table[decay_pot]; // set decay to pot setting
      }
      else {
        accent = accent_pot; // turn on accents
        vcf_env_decay = 0.9972f; // shorten vcf env decay to minimum
      }
    //} else if (step_counter == 7) {
    } else if (ppqn == 3) {
      if ((current_note & 0x80) == 0) vca_env_state = 3; // only turn off notes with no slide
    }
  }

  PORT->Group[0].OUTCLR.reg = 1<<23;
}