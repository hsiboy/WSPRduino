/*
  "DCF77decoder"
  Arduino-based DCF77 time signal decoder
  V2.3 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.
*/

#ifndef DCF77decoder_h_
#define DCF77decoder_h_

#if (ARDUINO >= 100)
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

// time.c - low level time and date functions
// Copyright (c) Michael Margolis 2009-2014
#include<Time.h>
#include <TimeLib.h>

// Reads and writes single bits at an array of type "unsigned char"
#include<BitArray.h>

#include <limits.h>
#include <stdint.h>
#include <util/atomic.h>

//################################################################################################################
//definitions
//################################################################################################################

class DCF77decoderClass {

public:

  void init(uint8_t pin, uint8_t mode);
  void setLocalTimezone(int8_t offset);
  uint32_t getSyncLoops();
  time_t getLastSync();

private:

  void syncTime();
  uint8_t parityCheck(uint8_t *da, uint8_t pos, uint8_t length);
  void swapDatasets();
  uint8_t fillTD();
  void DCF77isr();
  static void isr();

//################################################################################################################

  static DCF77decoderClass *this_instance;

// Flag holding the init-state of this routine
  uint8_t is_initialized = 0;

// Arrays holding the date and time information as derived from the DCF77-signal.
// td[3] => minute (1st digit)
// td[2] => minute (2nd digit)
// td[1] => hour (1st digit)
// td[0] => hour (2nd digit)

// td[9] => year (1nd digit)
// td[8] => year (2st digit)
// td[7] => month (1st digit)
// td[6] => month (2nd digit)
// td[5] => day (1st digit)
// td[4] => day (2nd digit)
// Set the date/time to 01.01.2000, 00:00.
  uint8_t td[10] = {0, 0, 0, 0, 0, 1, 0, 1, 0, 0};

// Flag that determines if the DCF77 date information should be obtained. If odat==0, only the time
// information will be obtained.
  uint8_t odat;

// Variable holding an offset in hours, if local time is different from DCF77 time (CET)
  volatile int8_t time_offset = 0;

// ... for the ISR

// A dummy to hold the timestamp of the last DCF-synchronisation.
  volatile time_t tsls = 0;
// Variable holding the millisecond-timestamp, when the DCF77 interrupt service routine was last called.
  uint32_t msl = 0;
// A dummy to hold the current millisecond-timestamp.
  uint32_t msc;
// Variable holding the time in milliseconds passed since the last call of the DCF77 interrupt service routine.
  uint32_t dt;
// two bit-arrays to hold the received DCF77 data of 2 consecutive minutes. The last bit (59) is the "data valid" flag.
  uint8_t da_a[8] = {0};
  uint8_t da_b[8] = {0};
// "pointer" to a bit-element of the above arrays
  uint8_t dap;
// a pointer to the current (in use) and old array
  uint8_t *da_current = da_a;
  uint8_t *da_old = da_b;
// Variable holding the synchronisation status of the DCF77-algorithm (1 -> is attempting to synchronize).
  uint8_t sync = 0;
// Variable holding the time passed in ms during the current DCF77-synchronisation cycle.
  uint32_t st;
// Flag indicating that a valid DCF77 data-set as been decoded and is avaliable.
  uint8_t dr = 0;
// Variable holding the number of successful synchronisations
  volatile uint32_t sync_loops = 0;

};

extern DCF77decoderClass DCF77;

#endif // DCF77decoder_h_

