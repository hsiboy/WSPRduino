"WSPR"
WSPR-library
V1.1 / Copyright (C) 2017, T.Rode (DL1DUZ)

Permission is granted to use, copy, modify, and distribute this software
and documentation for non-commercial purposes.

Acknowledgements:
  The WSPR message algorithm is based on the work of Andy Talbot, G4JNT.
  The code was derived from W3PM's project "Arduino UNO DDS-60/AD9850 WSPR
  /QRSS Controller".

Available functions:

"unsigned char encodeMessage(char call[], char locator[], unsigned long power)"
   Encodes WSPR-message and stores it in a 2bit array with 162 elements .
   
   The following constraints will be applied:
     - The callsign must have six characters consisting only of A-Z, a-z, 0-9 and [space].
     - The third character must always be a number. So, for example, "W1XY" must be
       given as "[sp]W1XY[sp]".
     - The locator must have 4 characters (such as JO61). The first two can each take on
       the values 'A' to 'R'.
       The remaining two can take the values '0' to '9'.
     - Power level must be given in milliwatts spanning from 1 to 1000000.

   Returns 1 (True) if coding was successful, 0 (False) otherwise.


"unsigned char getSymbol(unsigned char position)"
   returns the channel symbol(0...3) from a specified position (0...161) within the
   currently encoded WSPR-message.

Simple Arduino example code:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// requires "WSPR"- and "BitArray"-library
#include <WSPR.h>

void setup() {

// initialize serial communication to PC (via USB-Port)
  Serial.begin(9600);

// WSPR user data
  char call[] = "DL1DUZ";
  char locator[] = "JO61";
  unsigned char power = 200;

// encode user data; if successful, the encoded data will be sent to the PC, otherwise
// an error-message will be displayed
  if(WSPR.encodeMessage(call, locator, power)) {
    Serial.print("The symbols representing the call \"");
    Serial.print(call);
    Serial.print("\", the locator \"");
    Serial.print(locator);
    Serial.print("\" and the output power of ");
    Serial.print(power);
    Serial.println(" mW are:\n");

    for(unsigned char i=0; i<161; i++) {
      Serial.print(WSPR.getSymbol(i));
      Serial.print(", ");
    }
    Serial.print(WSPR.getSymbol(161));
  }
  else {
    Serial.print("The used data provided did not match the constraints.");
  }

}

void loop() { }
