/*
  "BitArray"
  Reads and writes single bits at an array of type "unsigned char"
  The size of this array must be at least 1+NUMBER_OF_BITS/8
  V0.4 / Copyright (C) 2016, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.
*/

#ifndef BitArray_h_
#define BitArray_h_

#include <stdint.h>

//################################################################################################################
//definitions
//################################################################################################################

class BitArrayClass {

public:

  uint8_t getBit(const uint8_t *bs, uint16_t bit);
  void setBit(uint8_t* bs, uint16_t bit, uint8_t value);
};

extern BitArrayClass BArray;

#endif // BitArray_h_

