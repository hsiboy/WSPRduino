/*
  "AD9850"
  library providing functions to control DDS modules based on AD9850 in serial mode (data pin == D7)
  V1.2 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Acknowledgements:
  Most algorithms were derived from Christophe Caiveaus library "Christophe Caiveau f4goj@free.fr" 
*/

//################################################################################################################
//includes
//################################################################################################################

#include <AD9850.h>

//################################################################################################################
//declarations
//################################################################################################################


//################################################################################################################
//constructor
//################################################################################################################

// W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN, CLOCK_FREQUENCY[Hz]
AD9850::AD9850(unsigned char w_clk_pin, unsigned char fq_ud_pin, unsigned char data_pin,
               unsigned char reset_pin, unsigned long clk_frequency) {
// assign and initialize AD9850 data pins
  W_CLK_PIN = w_clk_pin;
  FQ_UD_PIN = fq_ud_pin;
  DATA_PIN = data_pin;
  RESET_PIN = reset_pin;

  pinMode(W_CLK_PIN, OUTPUT);
  pinMode(FQ_UD_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

// turn on serial comm. mode
  ddsPulse(RESET_PIN);
  ddsPulse(W_CLK_PIN);
  ddsPulse(FQ_UD_PIN);

// set the system clock frequency
  FC = clk_frequency;

// turn off the DDS
  setPhase(0, 0, 1);
}

//################################################################################################################
//functions
//################################################################################################################

// support function that calculates the real DDS clock frequency by comparing a set output to the actually
// measured one; e.g. if a 125MHz-nominal DDS is set to 10MHz and the measured output is 9.9997MHz
// "calculateClockFrequency(10000000, 9999700, 125000000)" returns 124996250Hz = 124.99625MHz; for good
// numeric accuracy the test frequency should be chosen as high as possible
unsigned long AD9850::calculateClockFrequency(unsigned long f_set, unsigned long f_real, unsigned long clock) {
  uint64_t dummy = f_real*clock;
  dummy /= f_set;

  return (unsigned long) dummy;
}

//################################################################################################################

// calculates the phase-value for a given frequency in Hz
unsigned long AD9850::calculatePhaseValue(unsigned long frequency) {
  uint64_t dummy = frequency;
  dummy <<= 32;
  dummy /= FC;

  return (unsigned long) dummy; 
}

//################################################################################################################

// calculates the frequency in Hz for a give phase-value
unsigned long AD9850::calculateFrequency(unsigned long phase) {
  uint64_t dummy = phase;
  dummy *= FC;
  dummy >>= 32;

  return (unsigned long) dummy; 
}

//################################################################################################################

inline void AD9850::ddsPulse(uint8_t pin) {
  digitalWrite(pin, 1);
  digitalWrite(pin, 0);
}

//################################################################################################################

inline void AD9850::ddsShiftOut(uint8_t data) {
  shiftOut(DATA_PIN, W_CLK_PIN, LSBFIRST, data);
}

//################################################################################################################

// sets deltaphase (phase-stepwidth) and initial phase; initial phase coded on 5 bits (value ranging from 0...32,
// each step representing an angle of 11.25 degree)
// if shutdown is true, DDS will shut down, reducing the dissipated power from 380mW to 30mW @5V
void AD9850::setPhase(unsigned long deltaphase, unsigned char phase, boolean shutdown) {
  for (uint8_t i=0; i<4; i++, (deltaphase>>=8)) {
   ddsShiftOut(deltaphase & 0xFF);
  }
  ddsShiftOut((phase<<3) & 0xFF);
  ddsPulse(FQ_UD_PIN);
	
  if(shutdown) {
    if(!dds_is_down) {
      dds_is_down = 1;
      ddsPulse(FQ_UD_PIN);
      ddsShiftOut(0x04);
      ddsPulse(FQ_UD_PIN);
    }
  }
  else { dds_is_down = 0; }
}
