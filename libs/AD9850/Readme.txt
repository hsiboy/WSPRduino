"AD9850"
library providing functions to control DDS modules based on AD9850 in serial mode (data pin == D7)
V1.2 / Copyright (C) 2017, T.Rode (DL1DUZ)

Permission is granted to use, copy, modify, and distribute this software
and documentation for non-commercial purposes.

Acknowledgements:
Most algorithms were derived from Christophe Caiveaus library "Christophe Caiveau f4goj@free.fr" 

Available functions:

"void setPhase(unsigned long deltaphase, unsigned char phase, boolean shutdown)"
   sets deltaphase (phase-stepwidth) and initial phase; initial phase coded on 5 bits (value ranging from 0...32,
   each step representing an angle of 11.25 degree)
   if shutdown is true, DDS will shut down, reducing the dissipated power from 380mW to 30mW @5V

"unsigned long calculatePhaseValue(unsigned long frequency)"
   calculates the phase-value for a given frequency in Hz

"unsigned long calculateFrequency(unsigned long phase)"
   calculates the frequency in Hz for a give phase-value

"unsigned long calculateClockFrequency(unsigned long f_set, unsigned long f_real, unsigned long clock)"
   support function that calculates the real DDS clock frequency by comparing a set output to the actually
   measured one; e.g. if a 125MHz-nominal DDS is set to 10MHz and the measured output is 9.9997MHz
   "calculateClockFrequency(10000000, 9999700, 125000000)" returns 124996250Hz = 124.99625MHz; for good
   numeric accuracy the test frequency should be chosen as high as possible

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Arduino example code 1 (a wobble-generator, sweeping from 10.6MHz to 10.8MHz within 5s):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <AD9850.h>

// build AD9850-instance dds(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN, CLOCK_FREQUENCY[Hz])
// the exact clock frequency can be determined in a test-setup using the function
// "calculateClockFrequency" provided in this library
  AD9850 dds(7, 6, 5, 4, 125000000);
                                         
void setup() {}

void loop() {

  for(unsigned long f=10600000; f<10800001; f+=200) {
    dds.setPhase(dds.calculatePhaseValue(f), 0, false);
    delay(5);
  }

}

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Arduino example code 2 / for licensed radio amateurs only (a very, very simple 30m WSPR-beacon):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// requires "WSPR"-, "AD9850"-, "Time"- and "BitArray"-library
#include <WSPR.h>
#include <AD9850.h>
#include <Time.h>

// build AD9850-instance dds(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN, CLOCK_FREQUENCY[Hz])
// the exact clock frequency can be determined in a test-setup using the function
// "calculateClockFrequency" provided in this library
  AD9850 dds(7, 6, 5, 4, 125000000);

// Delta phase value representing 194Hz (200Hz - 6Hz signal bandwidth)
const unsigned int DELTAPHASE_BANDWITH = dds.calculatePhaseValue(194);

// Delta phase offset values (to represent 4bit PSK with 1.4648Hz spacing)
const unsigned char DELTAPHASE_PSK[4] = {0, uint8_t(dds.calculatePhaseValue(14648)/10000),
                                         uint8_t(dds.calculatePhaseValue(29296)/10000),
                                         uint8_t(dds.calculatePhaseValue(43944)/10000)};
                                         
void setup() {

// initialize serial communication to PC (via USB-Port) and wait for it to become available
  Serial.begin(9600);
  while (!Serial) {
    delay(1000);
  }

/*WSPR user data (call, locator, power[mW]
  The following constraints will be applied:
    - The callsign must have six characters consisting only of A-Z, a-z, 0-9 and [space].
    - The third character must always be a number. So, for example, "W1XY" must be
      given as "[sp]W1XY[sp]".
    - The locator must have 4 characters (such as JO61). The first two can each take on
      the values 'A' to 'R'.
      The remaining two can take the values '0' to '9'.
    - Power level must be given in milliwatts spanning from 1 to 1000000.
*/
  char call[] = "DL1DUZ";
  char locator[] = "JO61";
  unsigned char power = 200;

// encode user data; in case of an error, the system will be halted
  if(!WSPR.encodeMessage(call, locator, power)) {
    Serial.print("The used data provided did not match the constraints.");
    while(true) { delay(1000); }
  }

// init the random number generator
  randomSeed(0);

  Serial.println("Press 's' at zero seconds in an even minute to synchronize the clock and start the becon.");
  if(Serial.available() == 0 || Serial.read() != 's') {
    delay(100);
  }
  setTime(0);
}

void loop() {
// start transmission, if we are one second into an even minute
  if(!minute()%2 && second()==1) {
// set a random transmit frequency within the 200Hz bandwidth at 30m
    unsigned long deltaphase = dds.calculatePhaseValue(10140100) + random(0, DELTAPHASE_BANDWITH);

    Serial.print("Starting transmission on ");
    Serial.print(dds.calculateFrequency(deltaphase));
    Serial.println("Hz");
    
    for(unsigned char i = 0; i < 162; i++) {
// transmit symbol "i"
      dds.setPhase(deltaphase + DELTAPHASE_PSK[WSPR.getSymbol(i)], 0, false);
      delay(683);
    }
    dds.setPhase(0, 0, false);
    Serial.println("Transmission finished.");
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  }

  delayMicroseconds(1000);
}

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Arduino example code 3 / for licensed radio amateurs only (a little more sophisticated 160m-10m
WSPR-beacon applying task scheduling techniques for better usage of resources)

!!! some means of time synchronisation (GPS, DCF77, ...) controlling variable "tv" have to be
added to this sketch !!!

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// requires "WSPR"-, "AD9850"- and "Time"-library
#include<limits.h>

// Library providing basic WSPR functionality
#include <WSPR.h>
// Library providing basic AD9850 support
#include <AD9850.h>

#include <Time.h>

//##########################################################################################################

// create AD9850-instance dds(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN, (actual) CLOCK_FREQUENCY[Hz])
AD9850 dds(7, 6, 5, 4, 125000000);

// Variables/constants used by WSPR transmitter

// Flag indicating the status of the transmitter.
uint8_t on_air = 0;
// Flag indicating the status of the system time (time valid).
// ~~~~~~~~~~~~~~~~~ transmission only starts if tv is 1 /// this variable should be set by whatever synchronizes the time-library (GPS, DCF77, ...) ~~~~~~~~~~~~~~~~~~~
uint8_t tv = 1;

// the number of available bands (currently 10; 160-10m)
const uint8_t BAND_COUNT = 10;

// array holding the delta phase values for WSPR lower band limit @ 160m-10m
// can be adjusted depending on the actual DDS clock frequency by calling setDeltaPhaseBase()
uint32_t deltaphase_base[BAND_COUNT];

// the "band switch"
uint8_t band_pointer = 0;

// a pointer to the symbol table
uint8_t symbol_counter = 0;

// Delta phase value representing 194Hz (200Hz - 6Hz signal bandwidth)
const uint16_t DELTAPHASE_BANDWITH = dds.calculatePhaseValue(194);

// Delta phase offset values (to represent 4bit PSK with 1.4648Hz spacing)
const uint8_t DELTAPHASE_PSK[4] = {0, uint8_t(dds.calculatePhaseValue(14648)/10000),
                                   uint8_t(dds.calculatePhaseValue(29296)/10000),
                                   uint8_t(dds.calculatePhaseValue(43944)/10000)};

// the actual phaseword to set the DDS
uint32_t deltaphase;

// Variables/constants used by the repetitive task scheduler

// a constant specifying the target loop runtime in µs
// the smaller this number the more accurate the timing
// upper limit is 16383 due to software restrictions, lower limit is set by code runtime and
// CPU-speed (around 30µS for current 16MHz AVR)
// set to 50µs
const uint16_t TLR = 50;

// a constant specifying the loop cycles between 2 executions of the "system timer"-routine
// cycles = interval-time[µs]/TLR
// set to 100ms
const uint32_t SYSTEM_TIMER_LOOPS = 100000/TLR;
// the "system timer" loop cycle counter
uint32_t system_timer_loop_counter = 0;

// the constant specifying the loop cycles between 2 executions of the "single run task scheduler"-routine
// cycles = interval-time[µs]/TLR
// set to 1ms
const uint32_t SINGLE_RUN_TASK_SCHEDULER_LOOPS = 1000/TLR;
// "single run task scheduler" loop cycle counter
uint32_t single_run_task_scheduler_loop_counter = SINGLE_RUN_TASK_SCHEDULER_LOOPS;

// a constant specifying the loop cycles between 2 executions of the "transmit symbol"-routine
// cycles = interval-time[µs]/TLR
// set to 683ms
const uint32_t TRANSMIT_SYMBOL_LOOPS = 682667/TLR;
// the "transmit symbol" loop cycle counter
int32_t transmit_symbol_loop_counter = 0;

// variables used by the single run task scheduling routine

// a timer counting the seconds after system-startup; resolution is 0.1s (10 == 1s)
uint32_t timer = 0;
// a constant holding the max. task-id (in this example 4 tasks from 0...3 are available)
const uint8_t MID = 3;
// an array holding the due-time for all scheduled tasks (<0 == off/no execution)
int32_t tl[MID + 1];

//##########################################################################################################
//##########################################################################################################

void setup() {
// Compute WSPR message
  char call[7] = " G1VQI";
  char locator[5] = "XY12";
  uint32_t power = 200;
  
  if(!WSPR.encodeMessage(call, locator, power)) {
    // gets stuck here if user-provided data does not meet restrictions
    delay(1000);
  }

// base-frequencies for all bands
  deltaphase_base[0] = 1838000;
  deltaphase_base[1] = 3594000;
  deltaphase_base[2] = 5288600;
  deltaphase_base[3] = 7040000;
  deltaphase_base[4] = 10140100;
  deltaphase_base[5] = 14097000;
  deltaphase_base[6] = 18106000;
  deltaphase_base[7] = 21096000;
  deltaphase_base[8] = 24926000;
  deltaphase_base[9] = 28126000;
  
// Initially calculate and set the delta-phase values for WSPR lower band limits @ 160-10m
  for(uint8_t i=0; i<BAND_COUNT; i++) {
    deltaphase_base[i] = dds.calculatePhaseValue(deltaphase_base[i]);
  }

// init single run task array
  for(uint8_t i=0; i<=MID; i++) {
    tl[i] = -1;
  }

// Schedule switching of transmitter
  scheduleTask(0, 0);

// Schedule call of "randomSeed()"
  scheduleTask(1, 0);
}

//##########################################################################################################
//##########################################################################################################

void loop() {

// variables and constants used to calculate the loop - runtime
  static uint32_t stp = micros();
  static uint32_t st;
  static int32_t dt = 0;

  const int32_t MIN_DT = LONG_MIN/2;

// 1st cyclical task
// trigger "transmit symbol" execution (runs every 683ms)
  if(transmit_symbol_loop_counter == TRANSMIT_SYMBOL_LOOPS) {
    if(on_air) {
    // terminate transmission after 162 elements have been sent (110.6s)
      if(symbol_counter == 162) {
        setPhaseValue(0);
      }
    else {
        // transmit symbol given by "symbol_counter"
        setPhaseValue(deltaphase + DELTAPHASE_PSK[WSPR.getSymbol(symbol_counter)]);
        symbol_counter++;
      }
    }

    // task has been executed -> set counter to 0
    transmit_symbol_loop_counter = 0;
  }

// 2nd cyclical task
// trigger "system timer" execution (runs every 100ms /// time basis for single run task scheduler)
  if(system_timer_loop_counter == SYSTEM_TIMER_LOOPS) {
    timer++;
    // task has been executed -> set counter to 0 
    system_timer_loop_counter = 0;
  }
  
/* 3rd cyclical task
trigger "single run task scheduler" execution (runs every 1ms)
tasks can schedule other tasks or themselves from within this
routine; if a task reschedules itself, it becomes cyclical
until stopped externally by "scheduleTask(id, -1)"
*/
  if(single_run_task_scheduler_loop_counter == SINGLE_RUN_TASK_SCHEDULER_LOOPS) {
// scan array and execute due tasks
    for(uint8_t i=0; i<=MID; i++) {
      if(tl[i] > -1 && tl[i] <= timer) {
        // task i is due; mark task as done and execute it
        tl[i] = -1;
          
        switch(i) {
// execute task 0 -> switching of transmitter
          case 0:
            // Perform some checks before initiating transmission sequence:
            // Check if time is 1s into an even minute, if there is no ongoing transmission and if
            // the system time is valid
            if(!on_air && second()==1 && !(minute()%2) && tv) {
              // calculate a random transmit frequency
              deltaphase = deltaphase_base[band_pointer] + random(0, DELTAPHASE_BANDWITH);

              // switch to next rf-band (for next run)
              band_pointer = (band_pointer == (BAND_COUNT - 1)) ? 0 : band_pointer + 1;

              // set transmit symbol task to execute immediately
              transmit_symbol_loop_counter = TRANSMIT_SYMBOL_LOOPS;              
        
              // set "on air" flag for transmission to start
              on_air = 1;
            }

            scheduleTask(0, 1); // Re-schedule this task to run again in 100ms
          break;
// execute task 1 -> set new seed for the random number generator
          case 1:
            randomSeed(now());
            scheduleTask(1, 36000); // Re-schedule in 1h
          break;
// execute task 2 -> currently not used
          case 2:
// ~~~~~~~~~~~~~~~~~~~~~~~~ enter your code here ~~~~~~~~~~~~~~~~~~~~~~~~~
          break;
// execute task 3 -> currently not used
          case 3:
// ~~~~~~~~~~~~~~~~~~~~~~~~ enter your code here ~~~~~~~~~~~~~~~~~~~~~~~~~
          break;
        }
      }
    }

    // task has been executed -> set counter to 0
    single_run_task_scheduler_loop_counter = 0;
  }
  
// increment loop counter of all cyclical tasks
  transmit_symbol_loop_counter++;
  system_timer_loop_counter++;
  single_run_task_scheduler_loop_counter++;

// loop delay handling
// insures that on average the target loop runtime will be met
// delays due to code execution (e.g. when processing scheduled tasks) will be taken into account
  
// check if current system time is bigger then the timestamp obtained at the last loop
  dt += TLR;
  st = micros();
  if(st > stp) { dt -= int32_t(st - stp); }
  else { dt -= int32_t((ULONG_MAX - stp) + st + 1); }
  stp = st;
  
// if runtime-budget is larger then target, insert delay-step
  if(dt > TLR) {
    delayMicroseconds(dt);
    stp += dt;
    dt = 0;
  }
  else {
// avoid overflow, in case the budget is constantly negative (loop is too slow to meet target loop time)
    if(dt < MIN_DT) { dt = 0; }
  }
}

//##########################################################################################################
// general purpose functions
//##########################################################################################################

// initializes an array of the type uint8_t with a given number
void initArray(uint8_t *target, uint8_t target_size, uint8_t value) {
  for(uint8_t i=0; i<target_size; i++) {
    *(target+i) = value;
  }
}

//##########################################################################################################
// functions used for task scheduling
//##########################################################################################################

// (un)schedules a single run task
// "delay" should be given in 100ms resolution (10 == 1s; 0 == immediate execution; <0 == off/unschedule)
void scheduleTask(uint8_t id, int32_t delay) {
// check validity of task-id
  if(id <= MID) {

    if(delay > -1) {
      // schedule task by entering due-time into task list
      tl[id] = timer + delay;
    }
    else {
      // unschedule task
      tl[id] = -1;
    }
  }
}

//##########################################################################################################
// functions used for AD9850 control
//##########################################################################################################

// sets phase and restarts DDS
// if deltaphase is set to 0, DDS will shut down, reducing the dissipated power from 380mW to 30mW @5V
void setPhaseValue(uint32_t deltaphase) {
  dds.setPhase(deltaphase, 0, false);
  // reset WSPR if deltaphase is 0
  if(!deltaphase) {
    symbol_counter = 0;
    on_air = 0;
  }
}
