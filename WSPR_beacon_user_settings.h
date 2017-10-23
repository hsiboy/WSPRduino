// Beginning user provided data

// call, locator, power[mW]
const char CALL[7] = "DL1DUZ";  // 6 character callsign; 3rd character is forced to be a number; fill all
                                // blanks with <SPACE>
const char LOCATOR[5] = "JO61"; // Use 4 character locator e.g. "EM64"
const uint16_t POWER = 200;     // Power[mW] from 1 to 9999

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// the number of available bands (currently 10; 160-10m)
const uint8_t BAND_COUNT = 10;

// base-frequencies for all bands
const uint32_t BASE_FREQUENCY[BAND_COUNT] PROGMEM = {1838000,3594000,5366100,7040000,10140100,14097000,18106000,
                                                     21096000,24926000,28126000};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// status of the bands (true=enabled/false=disabled) from 160m (left) to 10m (right)
//uint8_t band_status[BAND_COUNT] = {true, true, true, true, true, true, true, true, true, true};
uint8_t band_status[BAND_COUNT] = {true, true, true, true, true, true, true, true, true, true};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
// beacon operating mode /// 0 = TX frequency will be randomly chosen within the band limits at each new
// transmission cycle; 1...194 = TX frequency will be fixed at "lower band limit" + 1...194 Hz
uint8_t beacon_mode = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/* beacon duty cycle
1 = 100% = transmits every even minute
2 = 50% = transmits every 2nd even minute
3 = 33.3% = transmits every 3rd even minute
4 = 26.6%
5 = 20%
6 = 16.6%
8 = 13.3%
10 = 10%
15 = 6.6%
30 = 3.3%
*/
uint8_t duty_cycle = 1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// the maximum permissive age [hours] of the last timestamp before beacon transmission is halted (1...255)
uint8_t max_unsynchronized = 9;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// the (actual) DDS clock frequency
const uint32_t DDS_CLK = 124999592;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// turn "SWR meter" on/off
const uint8_t SWR_METER_INSTALLED = false;

// sets the reference for analog inputs (INTERNAL = 1.1V, DEFAULT = 5V)
const uint8_t ref = DEFAULT;

// SWR-meter polynomial (correcting nonlinearities of the SWR-gauge / must be adjusted depending on the
// diodes used); SWR_x represents the output of the ADC and SWR_y the (corrected) value times 32
const uint16_t SWR_X[18] PROGMEM = {0,2,4,18,37,65,93,140,186,279,372,465,558,651,744,837,930,1023};
const uint16_t SWR_Y[18] PROGMEM = {0,261,442,1404,2441,3763,4954,6783,8436,11518,14365,17050,19612,22077,24460,26784,
                                    29760,32736};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// sets the timezone (offset to CET) /// (-1 = GMT, 0 = CET, 1 = EET)
const int8_t TIMEZONE = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// port assignment

// SWR-meter (analog pins)
const uint8_t SWR_REF_PIN = A4;
const uint8_t SWR_FWD_PIN = A5;

// Transmitter disabled (digital pin)
const uint8_t TRANSMITTER_DISABLED_PIN = 3;

// HD44780 (digital pins)
const uint8_t RS_PIN = 13;
const uint8_t ENABLE_PIN = 12;
const uint8_t D4_PIN = 11;
const uint8_t D5_PIN = 10;
const uint8_t D6_PIN = 9;
const uint8_t D7_PIN = 8;
const uint8_t LED = 1;

// DCF77 (digital pin capable of triggering interrupts)
const uint8_t DCF77_INPUT_PIN = 2;

// End user provided data
