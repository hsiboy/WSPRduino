"BitArray"
Reads and writes single bits at an array of type "uint8_t"
V0.4 / Copyright (C) 2016, T.Rode (DL1DUZ)

Permission is granted to use, copy, modify, and distribute this software
and documentation for non-commercial purposes.

Simple Arduino example code:
Create a bit-array holding 100 elements. Write and read/output values.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <BitArray.h>

//### Variables / Constants ########################################

  const uint16_t BIT_ARRAY_SIZE = 100;

// Define an array of the type "uint8_t" and with its size = 1+BIT_ARRAY_SIZE/8
  uint8_t bit_array[1+BIT_ARRAY_SIZE/8];

//### setup code (runs once) #######################################

void setup() {

// initialize serial communication to PC (via USB-Port)
  Serial.begin(9600);

// set the bits of the array intermittently to 0/1
  for(uint16_t i=0; i<BIT_ARRAY_SIZE; i++) {
    BArray.setBit(bit_array, i, i%2);
  }

}

//### loop code (runs continuously) #######################################

void loop() {

// output the result
  for(uint16_t i=0; i<BIT_ARRAY_SIZE; i++) {
    Serial.print("Bit ");
    Serial.print(i);
    Serial.print(" was set to ");
    Serial.print(BArray.getBit(bit_array, i));
    Serial.println(".");
  }
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

  delay(2000);
}

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
