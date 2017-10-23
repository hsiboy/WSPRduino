/*
  "WSPR"
  WSPR-library
  V1.2 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Acknowledgements:
  The WSPR message algorithm is based on the work of Andy Talbot, G4JNT.
  The code was derived from W3PM's project "Arduino UNO DDS-60/AD9850 WSPR/QRSS Controller".
*/

//################################################################################################################
//includes
//################################################################################################################

#include <WSPR.h>

//################################################################################################################
//declarations
//################################################################################################################


//################################################################################################################
//functions
//################################################################################################################

/* Encodes WSPR-message and stores it in an 162 bit binary array (symt[21])
   
   The following constraints will be applied:
     - The callsign must have six characters consisting only of A-Z, a-z, 0-9 and [space].
     - The third character must always be a number. So, for example, "W1XY" must be given as "[sp]W1XY[sp]".
     - The locator must have 4 characters (such as JO61). The first two can each take on the values 'A' to 'R'.
       The remaining two can take the values '0' to '9'.
     - Power level must be given in milliwatts spanning from 1 to 1000000.

   Returns 1 (True) if coding was successful, 0 (False) otherwise.
*/

unsigned char WSPRClass::encodeMessage(const char* call, const char* locator, unsigned long power) {

  char cl[7];
  for(uint8_t i=0; i<6; i++) {
    cl[i] = *(call + i);
  }
  char lo[5];
  for(uint8_t i=0; i<4; i++) {
    lo[i] = *(locator + i);
  }

// checking of input values and normalisation
  uint8_t is_valid = 0;

  if(normalizeCharacter(cl, 6) && normalizeCharacter(lo, 4) && cl[2] <= 9 && *(lo+2) <= 9 &&
     *(lo+3) <= 9 && *lo > 9 && *lo < 36 && *(lo+1) > 9 && *(lo+1) < 36 && power &&
     power <= 1000000) {

    is_valid = 1;

// Convert power level from mW to dBm. Only values ending in 0, 3 or 7 are permitted and will work
// with the WSJT / WSPR software.
    uint8_t d = 50;
    while(power < 100000) {
      power *= 10;
      d -= 10;
    }

    if(power <= 150000) { power = 0; }
    else {
      if(power <= 350000) { power = 3; }
      else {
        if(power <= 750000) { power = 7; }
        else { power = 10;
        }
      }
    } 

    power += d;

// coding of callsign and merging into message array ma[]
    uint8_t ma[11] = {0}; // encoded message

// coding of callsign
    uint32_t n1 = *(cl);
    n1 = n1*36 + *(cl+1);
    n1 = n1*10 + *(cl+2);
    n1 = n1*27 + *(cl+3) - 10;
    n1 = n1*27 + *(cl+4) - 10;
    n1 = n1*27 + *(cl+5) - 10;

// merge coded callsign into message array ma[]
    ma[0] = n1 >> 20;
    ma[1] = n1 >> 12;
    ma[2] = n1 >> 4;
    ma[3] = n1 << 4;

// coding of locator and power
    uint32_t m1=179-10*(*lo - 10) - *(lo+2);
    m1 = m1*180+10*(*(lo+1)-10) + *(lo+3);
    m1 <<= 7;
    m1 += power+64;

// merge coded locator and power into message array ma[]
    ma[3] += 0x0f & (m1>>18);
    ma[4] = m1 >> 10;
    ma[5] = m1 >> 2;
    ma[6] = m1 << 6;

// convolutional encoding of message array ma[] into a 162 bit stream
    uint8_t strm[21] = {0};    // temp symbol table
    uint16_t bc = 0, cnt = 0, cc = ma[0];
    uint32_t sh1 = 0;

    for (uint8_t i=0; i < 81; i++) {
      if (i % 8 == 0 ) {
        cc = ma[bc];
        bc++;
      }
      if(cc & 0x80) { sh1 = sh1 | 1; }

      BArray.setBit(strm, cnt++, checkParity(sh1 & 0xF2D05351));
      BArray.setBit(strm, cnt++, checkParity(sh1 & 0xE4613C47));

      cc <<= 1;
      sh1 <<= 1;
    }

// interleave reorder the 162 data bits and and merge table with the sync vector

/* 162 Bit synchronisation vector
  1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,1,1,0,0,1,
  1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,
  0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,
  0,0,0
*/
    uint8_t sync_vec[21] = {3,113,164,7,164,64,179,88,88,149,52,86,4,201,205,226,160,12,88,99,0};

    uint8_t ip=0;

    for (uint16_t i=0; i<256; i++) {
      uint16_t bis=1;
      uint8_t j=0;
      for (uint8_t b2=0; b2<8 ; b2++) {
        if (i & bis) { j |= 0x80 >> b2; }
        bis <<= 1;
      }
      if (j<162) {
        BArray.setBit(symt.sym_t_LSB, j, BArray.getBit(sync_vec, j));
        BArray.setBit(symt.sym_t_MSB, j, BArray.getBit(strm, ip));
        ip++;
      }
    }
  }
  
  return is_valid;
}

//################################################################################################################

// returns the channel symbol(0...3) from a specified position (0...161) within the currently encoded WSPR-message
unsigned char WSPRClass::getSymbol(unsigned char position) {
  return BArray.getBit(symt.sym_t_LSB, position) + (BArray.getBit(symt.sym_t_MSB, position)<<1);
}

//################################################################################################################

// parity-check routine
uint8_t WSPRClass::checkParity(uint32_t x) {
  uint8_t p = 0;

  for(uint8_t i=0; i<32; i++) {
    p += x & 1;
    x >>= 1;
  }

  return p & 1;
}

//################################################################################################################

// performs syntax-check and normalisation of a character-array with elements 0...9, A...Z, Space in order 0..36
uint8_t WSPRClass::normalizeCharacter(char* array, uint8_t char_count) {
  uint8_t is_valid = 1;

  for (uint8_t i=0; i < char_count; i++) {
    if(*(array + i) == ' ') { *(array + i) = 36; }
    else {
      if (*(array + i) >= '0' && *(array + i) <= '9') { *(array + i) = *(array + i)-'0'; }
      else {
        if (*(array + i) >= 'A' && *(array + i) <= 'Z') { *(array + i) = *(array + i)-'A'+10; }
        else {
          if (*(array + i) >= 'a' && *(array + i) <= 'z') { *(array + i) = *(array + i)-'a'+10; }
          else {
            is_valid = 0;
            break;
          }
        }
      }
    }
  }

  return(is_valid);
}

WSPRClass WSPR;