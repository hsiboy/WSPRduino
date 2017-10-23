/*
  "WSPRlib"
  WSPR-library
  V1.2 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Acknowledgements:
  The on-chip generation of the WSPR message algorithm is the work of 
  Andy Talbot, G4JNT. The code was derived from W3PM's project
  "Arduino UNO DDS-60/AD9850 WSPR/QRSS Controller".
*/

#ifndef WSPR_h_
#define WSPR_h_

// Reads and writes single bits at an array of type "unsigned char"
#include <BitArray.h>

#include <stdint.h>

//################################################################################################################
//definitions
//################################################################################################################

class WSPRClass {

public:

  unsigned char encodeMessage(const char* call, const char* locator, unsigned long power);
  unsigned char getSymbol(unsigned char position);

private:

  uint8_t checkParity(uint32_t li);
  uint8_t normalizeCharacter(char* string, uint8_t size);

//################################################################################################################

// WSPR symbol table
  typedef struct {                     // 162 element 4-state symbol table
    uint8_t sym_t_LSB[21] = {0};
    uint8_t sym_t_MSB[21] = {0};
  } SymTable;
  SymTable symt;

};

extern WSPRClass WSPR;

#endif // WSPR_h_

