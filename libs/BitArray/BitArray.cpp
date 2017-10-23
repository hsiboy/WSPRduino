/*
  "BitArray"
  Reads and writes single bits at an array of type "unsigned char"
  The size of this array must be at least 1+NUMBER_OF_BITS/8
  V0.4 / Copyright (C) 2016, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.
*/

//################################################################################################################
//includes
//################################################################################################################

#include <BitArray.h>

//################################################################################################################
//declarations
//################################################################################################################

//##########################################################################################################
//functions
//##########################################################################################################

// returns a single bit from a bit-array of the type unsigned char
uint8_t BitArrayClass::getBit(const uint8_t* bs, uint16_t bit) {
  return (*(bs+(bit>>3)) >> bit%8) & 1;
}

//##########################################################################################################

// sets a single bit at a bit-array of the type unsigned char
void BitArrayClass::setBit(uint8_t* bs, uint16_t bit, uint8_t value) {

  uint8_t dummy_1 = 1 << bit%8;
  uint8_t *dummy_2 = bs + (bit>>3);

// reset bit n
  *dummy_2 -= (*dummy_2 & dummy_1);
// set bit n, depending on "value"
  if(value) { *dummy_2 += dummy_1; }
}

BitArrayClass BArray;
