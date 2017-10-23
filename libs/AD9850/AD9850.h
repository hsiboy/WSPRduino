/*
  "AD9850"
  library providing functions to control DDS modules based on AD9850 in serial mode (data pin == D7)
  V1.2 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Acknowledgements:
  Most algorithms were derived from Christophe Caiveaus library "Christophe Caiveau f4goj@free.fr" 
*/

#ifndef AD9850_h_
#define AD9850_h_

#if (ARDUINO >= 100)
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

#include <stdint.h>

//################################################################################################################
//definitions
//################################################################################################################

class AD9850 {

public:
// constructor
  AD9850(unsigned char W_CLK_PIN, unsigned char FQ_UD_PIN, unsigned char DATA_PIN, unsigned char RESET_PIN,
         unsigned long clk_frequency);
// destructor
  ~AD9850() { }

// sets deltaphase (phase-stepwidth) and initial phase; initial phase coded on 5 bits (value ranging from 0...32,
// each step representing an angle of 11.25 degree)
// if deltaphase is set to 0, DDS will shut down, reducing the dissipated power from 380mW to 30mW @5V
  void setPhase(unsigned long deltaphase, unsigned char phase, boolean shutdown);
// calculates the phase-value for a given frequency in Hz
  unsigned long calculatePhaseValue(unsigned long frequency);
// calculates the frequency in Hz for a give phase-value
  unsigned long calculateFrequency(unsigned long phase);
// support function that calculates the real DDS clock frequency by comparing a set output to the actually
// measured one; e.g. if a 125MHz-nominal DDS is set to 10MHz and the measured output is 9.9997MHz
// "calculateClockFrequency(10000000, 9999700, 125000000)" returns 124996250Hz = 124.99625MHz; for good
// numeric accuracy the test frequency should be choosen as high as possible
  unsigned long calculateClockFrequency(unsigned long f_set, unsigned long f_real, unsigned long clock);

private:
// AD9850 data pins
  uint8_t W_CLK_PIN;
  uint8_t FQ_UD_PIN;
  uint8_t DATA_PIN;
  uint8_t RESET_PIN;
// status-flag (if DDS is shut down, dissipated power reduces from 380mW to 30mW @5V)
  uint8_t dds_is_down;
// actual DDS clock frequency [Hz] which might differ from 125 MHz
  uint32_t FC;

  inline void ddsPulse(uint8_t pin);
  inline void ddsShiftOut(uint8_t data);
};

#endif // AD9850_h_

