"DCF77decoder" / Arduino-based DCF77 time signal decoder
  V2.3 / Copyright (C) 2017, T.Rode (DL1DUZ)

Requirements:
// time.c - low level time and date functions
// Copyright (c) Michael Margolis 2009

This library allows the reception of the DCF77 time signal transmitter (atomic clock).
Once a valid timestamp has been obtained, the system clock will be synchronized by
means provided by the "Time"-library (added to this package / latest version to be
found at the arduino-playground).

Arduino example code:
Sends the synchronized Time/Date to the PC via USB connection and Serial Monitor

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// time.c - low level time and date functions
// Copyright (c) Michael Margolis 2009-2014
#include<Time.h>

// Arduino-based DCF77 time signal decoder
#include <DCF77decoder.h>

//### Variables / Constants ########################################

/* DCF77_INPUT
   the settings depend on the board you use; the port must be capable triggering
   interrupts; the value below (port D2) was used at an "Arduino Leonardo"

   MODE (false -> only time will be received; true -> time and date will be received)
*/

  const byte DCF77_INPUT = 2;
  const boolean MODE = true;

// just a dummy needed for this demo
  int dummy = 0;

//### setup code (runs once) #######################################

void setup() {

// initialization the DCF77 decoder; to be called once; subsequent calls will have no effect
   DCF77.init(DCF77_INPUT, MODE);
// set the local timezone to CET - 1h (== GMT)
   DCF77.setLocalTimezone(-1);

// initialize serial communication to PC (via USB-Port)
  Serial.begin(9600);
  Serial.println("Waiting for system clock to synchronize with DCF77.\n");

}

//### loop code (runs continuously) #######################################

void loop() {

  if(DCF77.getSyncLoops()) {
    if(second() != dummy) {
      dummy = second();

      Serial.print("Last synchronisation: ");
      Serial.print(hour(DCF77.getLastSync())); Serial.print(":");
      Serial.print(minute(DCF77.getLastSync())); Serial.print(":");
      Serial.println(second(DCF77.getLastSync()));Serial.println("\n");

      Serial.print("Current time is: ");
      Serial.print(hour()); Serial.print(":");
      Serial.print(minute()); Serial.print(":");
      Serial.print(second()); Serial.print("   Date is: ");
      Serial.print(day()); Serial.print(".");
      Serial.print(month()); Serial.print(".");
      Serial.println(year());
      Serial.print("\n############################################\n");
    }
  }

  delay(100);

}

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
