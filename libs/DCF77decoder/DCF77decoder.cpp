/*
  "DCF77decoder"
  Arduino-based DCF77 time signal decoder
  V2.3 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.
*/

//################################################################################################################
//includes
//################################################################################################################

#include <DCF77decoder.h>

//################################################################################################################
//declarations
//################################################################################################################

DCF77decoderClass *DCF77decoderClass::this_instance;

//################################################################################################################
//functions
//################################################################################################################

// initializes the DCF77 receiver
// this function is to be called once to activate DCF77-synchronisation; subsequent calls will have no effect

// pin -> processor-pin, connected to DCF77-receiver (must be capable of triggering interrupts)
// mode -> receiver-mode (0 -> only time will be synchronized; 1 -> time and date will be synchronized)

void DCF77decoderClass::init(uint8_t pin, uint8_t mode) {
  if(!is_initialized) {
    pinMode(pin, INPUT);
    this_instance = this;
    attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
    syncTime();
    odat = mode;
    is_initialized = 1;
  }
}

//################################################################################################################

void DCF77decoderClass::isr() {
  this_instance -> DCF77isr();
}

//################################################################################################################

// changes the offset [h] between local time and DCF77-time (default is 0)

void DCF77decoderClass::setLocalTimezone(int8_t offset) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    time_offset = offset;
  }
}

//################################################################################################################

// returns the number of successful synchronisations

uint32_t DCF77decoderClass::getSyncLoops() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    return sync_loops;
  }
}

//################################################################################################################

// returns the timestamp of the last DCF-synchronisation as a time_t - variable (0 == never synchronized)
time_t DCF77decoderClass::getLastSync() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    return tsls;
  }
}

//################################################################################################################

// synchronizes the time and date ("Time"-library) based on the data stored in the td-array. An offset [h] can be
// applied

void DCF77decoderClass::syncTime() {
  tmElements_t t;

  t.Year = 10*td[8] + td[9] + 30;
  t.Month = 10*td[6] + td[7];
  t.Day = 10*td[4] + td[5];
  t.Hour = 10*td[0] + td[1];
  t.Minute = 10*td[2] + td[3];
  t.Second = 0;

  setTime(makeTime(t) + 3600*time_offset);
}

//################################################################################################################
  
// performs a parity check on the DCF77 data; returns "1" if data has even parity

// *da -> pointer to the first element
// pos -> start-position
// length -> number of elements to be processed

uint8_t DCF77decoderClass::parityCheck(uint8_t *da, uint8_t pos, uint8_t len) {
  uint8_t pok = 1;

  for(uint8_t i=pos; i<pos+len; i++) {
    pok = BArray.getBit(da, i)^pok;
  }

  return pok;
}

//################################################################################################################
  
// swap datasets (*current <=> *old)

void DCF77decoderClass::swapDatasets() {
  uint8_t *dummy = da_current;
  da_current = da_old;
  da_old = dummy;
}

//################################################################################################################

// fills the td-array and performs a plausibility-check (past cycle vs. current)
// returns 1, if ok

uint8_t DCF77decoderClass::fillTD() {
  // constants holding the position and length of the data in the received set
  const uint8_t source[10] = {33,29,25,21,40,36,49,45,54,50};
  const uint8_t length[10] = {2,4,3,4,2,4,1,4,4,4};
  //... to conrol the number of elements to be extracted (with or without date)
  uint8_t elements = 4;
  if(odat) { elements = 10; }
  
  uint8_t is_valid = 1;
  
  for(uint8_t i=0; i<elements; i++) {
    uint8_t current = 0;
	uint8_t old = 0;
  
    for(uint8_t j=source[i]; j<source[i]+length[i]; j++) {
      BArray.setBit(&current,  j-source[i], BArray.getBit(da_current, j));
	  BArray.setBit(&old,  j-source[i], BArray.getBit(da_old, j));
    }
	// fill td[i]
	td[i] = current;
	// add one minute to old dataset
	if(i==3) {old++;}
	// terminate processing, if there is a mismatch between current and old dataset
	if(current!=old) {
	  is_valid = 0;
	  break;
	}
  }
  return is_valid;
}

//################################################################################################################

//DCF77 interrupt service routine

void DCF77decoderClass::DCF77isr() {

// Catch the current millisecond-timestamp when entering the interrupt service routine.
  msc = millis();
// calculate the time in ms passed since the last call of this routine.
  dt = msc - msl;

// check min. pulse length to be >75ms (improves robustness)
  if(dt > 75) {
// Update the variable holding the time index of the last call of this routine.
    msl = msc;
// synchronisation mode is on
    if(sync) {
// increment synchronisation timer
      st += dt;

// Cancel the current synchronisation attempt, if it lasted more than 1min and 4s.
      if(st > 64000) {
        sync = 0;
      }
      else {
// check, if the time-gap equals a 1 or 0 information (75ms...175ms == 0; 176ms...275ms == 1)
        if(dt < 276) {
		  // equals 0
		  uint8_t bit_value = 0;
          if(dt > 175) {
            // equals 1
            bit_value = 1;
          }
		  BArray.setBit(da_current, dap, bit_value);
		  dap++;

// a full set of data has been received (elements 0...58 read)
          if(dap == 59) {
	        // set data-valid flag at current dataset
	        BArray.setBit(da_current, dap, (parityCheck(da_current, 21, 8) && parityCheck(da_current, 29, 7) && 
                          (!odat || parityCheck(da_current, 36, 23))));
		  
	        // check if received data (current dataset) is valid
	        if(BArray.getBit(da_current, dap)) {
	          // check if old dataset is valid too
	          if(BArray.getBit(da_old, dap)) {
                // Load the "td"-array with the current data and perform plausibility-checks (current vs. old)
	            if(fillTD()) {
		        // All checks were fine. Set the "data received" flag.
                  dr = 1;
		          // invalidate old dataset
		          da_old[7] = 0;
		          // swap current and old
		          swapDatasets();
	            }
	            else {
		        // invalidate both datasets and start from scratch
		          da_old[7] = 0;
		          da_current[7] = 0;
	            }
	          }
              else {
	            // swap current and old
	            swapDatasets();
	          }
	        }
	        else {
	          // invalidate old dataset and start from scratch
	          da_old[7] = 0;
	        }
	        sync = 0;
	      }
	    }
      }
    }
// synchronisation mode is off
    else {
// Check if the time-gap (last call to now) equals the minute-mark
      if(dt > 1724 && dt < 1925) {
// If data stored in the td[] arrays is valid, synchronize the system time ("Time"-library).
        if(dr) {
          syncTime();
// Set the synchronisation timestamp
          tsls = now();
// Increase the "successful synchronisations" counter
          sync_loops++;
        }
// Reset the data array pointer.
        dap = 0;
// Reset the runtime of the current synchronisation cycle in milliseconds.
        st = 0;
// Re-enable synchronisation to start next cycle.    
        sync = 1;
      }
      dr = 0;
    }
  }
}

DCF77decoderClass DCF77;
