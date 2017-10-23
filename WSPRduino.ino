/*
 WSPR_beacon
 V1.5
 Copyright (C) 2017
 Thomas Rode / DL1DUZ
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 Arduino-based WSPR beacon
 
 Generates WSPR coordinated frequency hopping transmissions 
 on 10 thru 160 meters using a AD9850 DDS.

 Time synchronisation is done by a DCF77 time-signal receiver.
*/

#include <stdint.h>
#include<limits.h>

// Library providing basic WSPR functionality
#include <WSPR.h>
#include <AD9850.h>
// Arduino-based DCF77 time signal decoder
#include <DCF77decoder.h>

#include <LiquidCrystal.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "WSPR_beacon_user_settings.h"

//##########################################################################################################

// build AD9850-instance dds(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN, CLOCK_FREQUENCY[Hz])
AD9850 dds(7, 6, 5, 4, DDS_CLK);

/* Variables/constants available at the LCD module
32  SP  64  @ 96  `
33  !  65 A 97  a
34  "  66 B 98  b
35  #  67 C 99  c
36  $  68 D 100 d
37  %  69 E 101 e
38  &  70 F 102 f
39  '  71 G 103 g
40  (  72 H 104 h
41  )  73 I 105 i
42  *  74 J 106 j
43  +  75 K 107 k
44  ,  76 L 108 l
45  -  77 M 109 m
46  .  78 N 110 n
47  /  79 O 111 o
48  0  80 P 112 p
49  1  81 Q 113 q
50  2  82 R 114 r
51  3  83 S 115 s
52  4  84 T 116 t
53  5  85 U 117 u
54  6  86 V 118 v
55  7  87 W 119 w
56  8  88 X 120 x
57  9  89 Y 121 y
58  :  90 Z 122 z
59  ;  91 [ 123 {
60  <  92 ¥ 124 |
61  =  93 ] 125 }
62  >  94 ^ 126 →
63  ?  95 _ 127 ←

253 ÷ 244 Ω 223 °
*/

// some LCD display messages
const uint8_t BAND_INFO[33] PROGMEM = "160 80 60 40 30  20 17 15 12 10 ";
const uint8_t SETTINGS_NOT_VALID[33] PROGMEM = "invalid settings/input discarded";
const uint8_t INTRO[33] PROGMEM = " WSPR-beacon by DL1DUZ  160m-10m";
const uint8_t USER_DATA[33] PROGMEM = "       Loc:     Power:    mW    ";
const uint8_t TIME_LAST_SYNC[33] PROGMEM = "latest timestamp  :      .  .   ";
const uint8_t TX_STANDBY[33] PROGMEM = "TX is on standby                ";
const uint8_t TX_ON_AIR[33] PROGMEM = "TX is on air at    m / SWR:-.-  ";
const uint8_t TX_SWR_DISABLED[33] PROGMEM = "TX disabled due to SWR>3 at    m";
const uint8_t TX_TIME_DISABLED[33] PROGMEM = "TX disabled due to invalid time ";
const uint8_t TX_HARDWARE_DISABLED[33] PROGMEM = " TX disabled by hardware switch ";
const uint8_t TIME_NOT_SYNC[33] PROGMEM = "obtaining time &date from DCF77 ";
const uint8_t TIME_DATE[33] PROGMEM = "  ---. --.--.--  Time: --:--:-- ";
const uint8_t SYSTEM_HALTED[33] PROGMEM = "system halted byhardware switch ";
const uint8_t WEEKDAY_CODING[22] PROGMEM = "SunMonTueWedThuFriSat";

// build LiquidCrystal-instance
LiquidCrystal lcd(RS_PIN, ENABLE_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

// the number of characters at the LCD-display
const uint8_t DISPLAY_SIZE = 32;

// the content matrix lcd_content[position] and its last image
// character 0...15 for line 1 and 16...31 for line 2
uint8_t lcd_content[DISPLAY_SIZE];
uint8_t lcd_content_last[DISPLAY_SIZE] = {0};

// the current cursor position
uint8_t cursor_position = 0;
// current cursor status (0==nothing; 1==only corsor; 2==only blinking; 3==blinking cursor)
uint8_t cursor_status = 0;

// a pointer determining the active display content and its max. value
const uint8_t MAX_DISP_CONTENT = 2;
uint8_t disp_content_pointer = 0;
// a pointer determining the content to be on fast refresh
uint8_t fast_refresh_pointer = 0;

// the current LED-backlight brightness, the min value and change per step
const uint8_t MIN_BRIGHTNESS = 30;
uint8_t brightness;
int8_t brightness_change;
uint8_t pulsing_on;
uint8_t backlight_on_time_counter;

// Variables/constants used by WSPR transmitter

time_t last_sync = 0;

// Flags indicating the status of the transmitter (active and hardware-disabled).
uint8_t on_air;
uint8_t td;
uint8_t td_acknowledged = 0;
// Flag indicating that the system time is valid
uint8_t tv = 0;
// Array holding the coded band-information to be displayed on LCD
char band[4];
// array holding the delta phase values for WSPR lower band limit @ 160m-10m
// can be adjusted depending on the actual DDS clock frequency by calling setDeltaPhaseBase()
uint32_t deltaphase_base[BAND_COUNT];
// variables holding the reflected SWR-readings
uint16_t fwd_value;
uint32_t ref_value;
// array holding the 10x the SWR measured at the various bands (0 == SWR was >3 at more than 1 run)
uint8_t swr[BAND_COUNT];
// sum of all SWR-values
uint16_t swr_sum = BAND_COUNT;
// swr sliding average over 10 measurements and a pointer to the current value
const uint8_t SWR_AVG_LENGTH = 10;
uint8_t swr_avg[SWR_AVG_LENGTH];
uint8_t swr_avg_pointer;
// Flag indicating that the SWR-meater is active
uint8_t SWR_check_active = 0;

// the "band switch"
uint8_t band_pointer = BAND_COUNT - 1;

// a pointer to the symbol table
uint8_t symbol_counter;

// Delta phase value representing 194Hz (200Hz - 6Hz signal bandwidth)
const uint16_t DELTAPHASE_BANDWITH = dds.calculatePhaseValue(194);

// Delta phase offset values (to represent 4bit PSK with 1.4648Hz spacing)
const uint8_t DELTAPHASE_PSK[4] = {0, uint8_t(dds.calculatePhaseValue(14648)/10000),
                                   uint8_t(dds.calculatePhaseValue(29296)/10000),
                                   uint8_t(dds.calculatePhaseValue(43944)/10000)};

// the actual phaseword to set the DDS
uint32_t deltaphase;

// Variables/constants used by the repetitive task scheduler

// a constant specifying the desired loop runtime in µs
// should not be bigger then 16383 due to software restrictions
// a proven value is about 100x the shortest task interval
// set to 50µs
const uint8_t TLR = 50;

// a constant specifying the loop cycles between 2 executions of the "system timer"-routine
// cycles = interval-time[µs]/TLR
// set to 100ms
const uint16_t SYSTEM_TIMER_LOOPS = 100000/TLR;
// the "system timer" loop cycle counter
uint16_t system_timer_loop_counter = 0;

// the constant specifying the loop cycles between 2 executions of the "single task scheduler"-routine
// cycles = interval-time[µs]/TLR
// set to 10ms
const uint8_t SINGLE_TASK_SCHEDULER_LOOPS = 10070/TLR;
// "single task scheduler" loop cycle counter
uint8_t single_task_scheduler_loop_counter = SINGLE_TASK_SCHEDULER_LOOPS;

// the constant specifying the loop cycles between 2 executions of the "check SWR"-routine
// cycles = interval-time[µs]/TLR
// set to 2.5ms
const uint16_t CHECK_SWR_LOOPS = 2500/TLR;
// "check SWR" loop cycle counter
int16_t check_SWR_loop_counter = 0;

// a constant specifying the loop cycles between 2 executions of the "transmit symbol"-routine
// cycles = interval-time[µs]/TLR
// set to 683ms
const uint16_t TRANSMIT_SYMBOL_LOOPS = 682667/TLR;
// the "transmit symbol" loop cycle counter
int16_t transmit_symbol_loop_counter = TRANSMIT_SYMBOL_LOOPS;

// variables used by the single task scheduling routine

// a timer counting the seconds after system-startup; resolution is 0.1s (10 == 1s)
volatile uint32_t timer = 0;
// a constant holding the max. task-id (in this example 6 tasks from 0...5 are available)
const uint8_t MID = 5;
// an array holding the due-time for all scheduled tasks (<0 == off/no execution)
volatile int32_t tl[MID + 1];

//##########################################################################################################
//##########################################################################################################

void setup() {
// initialize the DCF77 decoder; to be called once; subsequent calls will have no effect
// initDFC77(DCF77_INPUT_PIN, MODE, OFFSET);
// DCF77_INPUT_PIN -> input pin (connected to hardware receiver)
// MODE (false -> only time will be received; true -> time and date will be received)
  DCF77.init(DCF77_INPUT_PIN, 1);
  DCF77.setLocalTimezone(TIMEZONE);

  analogReference(ref);
  pinMode(SWR_FWD_PIN, INPUT);
  pinMode(SWR_REF_PIN, INPUT);

  pinMode(TRANSMITTER_DISABLED_PIN, INPUT);  

// initialize the HD44780 LCD modulel
  pinMode(RS_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(D4_PIN, OUTPUT);
  pinMode(D5_PIN, OUTPUT);
  pinMode(D6_PIN, OUTPUT);
  pinMode(D7_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  
  lcd.begin(16,2);

// Initially calculate and set the delta-phase values for WSPR lower band limits @ 160-10m
  for(uint8_t i=0; i<BAND_COUNT; i++) {
    deltaphase_base[i] = dds.calculatePhaseValue(pgm_read_dword_near(BASE_FREQUENCY + i));
  }

// init the content matrix lcd_content[position]
  initArray(lcd_content, DISPLAY_SIZE, 32);
// init the SWR-array
  initArray(swr, BAND_COUNT, 10);
// init task array
  for(uint8_t i=0; i<=MID; i++) {
    tl[i] = -1;
  }

// initially turn off DDS (only to initialize the display)
  setPhaseValue(0, 0);
  pulsing_on = 0;

// initialize timer 2
  cli(); // disable global interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // set entire TCCR2B register to 0
  OCR2A = 59; // set compare match register to trigger every 30µs
  TCCR2A |= (1 << WGM21);  // turn on CTC mode
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
  sei(); // enable global interrupts

  digitalWrite(LED, HIGH);
// Display intro   
  writeToBuffer(INTRO);
  loadLCD();
  delay(4000);
  
// code WSPR message & check band status (at least 1 must be active); in case of errors system will be halted
  uint8_t min_one_band_active = 0;
  for(uint8_t i=0; i<BAND_COUNT; i++) {
    min_one_band_active += band_status[i];
  }
  if(!WSPR.encodeMessage(CALL, LOCATOR, POWER) || !min_one_band_active) {
    writeToBuffer(SETTINGS_NOT_VALID);
    loadLCD();
    while(true) {
      digitalWrite(LED, LOW);
      delay(1000);
      digitalWrite(LED, HIGH);
      delay(1000);
    }
  }
   
// Display band status
  const char DISABLED[3] = {'-', '-', '-'};
  const uint8_t POS[10] = {0, 4, 7, 10, 13, 17, 20, 23, 26, 29};
  uint8_t len = 3;
// BAND_INFO_ADDRESS
  writeToBuffer(BAND_INFO);
  for(uint8_t i=0; i<BAND_COUNT; i++) {
    if(!band_status[i]) {
      if(i) { len = 2; }
      writeCharArray(DISABLED, POS[i], len);
    }
  }
  loadLCD();
  delay(4000);
   
// Display user settings   
  writeToBuffer(USER_DATA);
  writeCharArray(CALL, 0, 6);
  writeCharArray(LOCATOR, 11, 4);
  writeNumber(POWER, 21, 0, 4);
  loadLCD();
  delay(4000);
   
// halt system for time synchronisation if "transmitter disabled"-switch is on
  if(digitalRead(TRANSMITTER_DISABLED_PIN)) {
    writeToBuffer(SYSTEM_HALTED);
    loadLCD();
    while(digitalRead(TRANSMITTER_DISABLED_PIN)) { delay(10); }   
  // set system clock to 01.01.2000 00:00:00
    last_sync = 946684800;
    tv = 1;
    setTime(last_sync);
  }
  
  setPhaseValue(0, 0);

// Schedule switching of transmitter
  scheduleTask(0, 0);

// call randomSeed()  
  scheduleTask(1, 0);
 
}

//##########################################################################################################
//##########################################################################################################

// timer 2 compa-isr (backlight pulsing)
ISR(TIMER2_COMPA_vect) {
  if(!backlight_on_time_counter) {
    digitalWrite(LED, 1);

    if(brightness==255) { brightness_change = -1; }
    else {
      if(brightness==MIN_BRIGHTNESS) {
        brightness_change = 1;
        scheduleTask(5, 0);
      }
    }

    brightness+=brightness_change;
  }
  else {
    if(backlight_on_time_counter == brightness) { digitalWrite(LED, 0); }
  }
  backlight_on_time_counter++;
}

//##########################################################################################################
//##########################################################################################################

void loop() {

// variables and constants used to calculate the loop - runtime
  static uint32_t stp = micros();
  static uint32_t st;
  static int32_t dt = 0;

  const int32_t MIN_DT = LONG_MIN/2;

// trigger "transmit symbol" execution (runs every 683ms)
  if(transmit_symbol_loop_counter == TRANSMIT_SYMBOL_LOOPS) {
    if(on_air) {
      // terminate transmission after 110.6s
      if(symbol_counter == 162) {
        setPhaseValue(0, 0);
      }
      else {
        setPhaseValue(deltaphase + DELTAPHASE_PSK[WSPR.getSymbol(symbol_counter)], 0);
        symbol_counter++;
      }
    }
    // Transmitter is not on-air; use this slot to validate system time
    else {
      // check, if time has been synchronized and last synchronisation is not "older" than x hours
      if(last_sync && (now() - last_sync < max_unsynchronized*3600)) {
        tv = 1;
      }
      else {
        tv = 0;
      }
    }

    transmit_symbol_loop_counter = 0;
  }

// trigger "system timer" execution (runs every 100ms)
  if(system_timer_loop_counter == SYSTEM_TIMER_LOOPS) {
    // update the "last time synchronisation" timestamp
    if(DCF77.getLastSync()) {
	    last_sync = DCF77.getLastSync();
	  }
	
    timer++;   
    system_timer_loop_counter = 0;
  }

// trigger "check SWR" execution (runs every 2.5ms)
  if(check_SWR_loop_counter == CHECK_SWR_LOOPS) {
  // readout SWR-meter and check SWR to be <= 3.0
    if(SWR_check_active) {
    // wait until forward reading has reached 25% of max-value to avoid false triggering
      fwd_value = analogRead(SWR_FWD_PIN);
      if(fwd_value > 255) {
        ref_value = getPolyValue(analogRead(SWR_REF_PIN), 18, SWR_X, SWR_Y);
        ref_value <<= 16;
        ref_value /= fwd_value;
        if(ref_value > 1712353) { ref_value = 1712353; }
        // store current swr-value in swr-averaging array
        swr_avg[swr_avg_pointer] = (20971520 + 10*ref_value)/(2097152 - ref_value);

        if(swr_avg[swr_avg_pointer] > 30) {
          setPhaseValue(0, 0);
          if(swr[band_pointer] > 30) {
            swr_avg[swr_avg_pointer] = 0;
          }
          swr[band_pointer] = swr_avg[swr_avg_pointer];
        }

        // increase swr-average array pointer
        swr_avg_pointer = (swr_avg_pointer == (SWR_AVG_LENGTH - 1)) ? 0 : swr_avg_pointer + 1;
      }
    }
  
    check_SWR_loop_counter = 0;
  }

/* trigger "single task scheduler" execution (runs every 10ms)

tasks can schedule other tasks or themselves from within this
routine; if a task reschedules itself, it becomes cyclical
until stopped externally by "scheduleTask(id, -1)"
*/
  if(single_task_scheduler_loop_counter == SINGLE_TASK_SCHEDULER_LOOPS) {
// scan array and execute due tasks
    for(uint8_t i=0; i<=MID; i++) {
      if(tl[i] > -1 && tl[i] <= timer) {
        tl[i] = -1;
          
        switch(i) {
// execute task 0 -> switching of transmitter
          case 0:
            // set "transmitter disabled" flag
            td = digitalRead(TRANSMITTER_DISABLED_PIN);
            // Shutdown WSPR transmitter and reset SWR-readings, if transmitter-disabled flag was set
            // In this mode also the setup-conrols are active
            if(td) {
              if(!td_acknowledged) {
                setPhaseValue(0, 1);
                // init the SWR-array
                initArray(swr, BAND_COUNT, 10);
                td_acknowledged = 1;
              }
            }
            else {
              if(td_acknowledged) { td_acknowledged = 0; }
              /* Do some checks before initiating transmission sequence:
                Check that there is no ongoing transmission, that time is 0s into an even minute,
                that there is at least one band with an SWR < 3 and that the system time is valid
              */
              swr_sum = 0;
              for(uint8_t i=0; i<BAND_COUNT; i++) {
                if(band_status[i]) { swr_sum += swr[i]; }
              }

              if(!on_air && !second() && !(minute()%(duty_cycle<<1)) && swr_sum && tv) {
                // delay transmission by 950ms (should actually start 1s into the even minute)
                transmit_symbol_loop_counter = -267333/TLR;

                // schedule SWR-readout to start 2.5ms after the beacon
                if(SWR_METER_INSTALLED) {
                  check_SWR_loop_counter = -950000/TLR;
                  SWR_check_active = 1;
                  initArray(swr_avg, SWR_AVG_LENGTH, 10);
                  swr_avg_pointer = 0; 
                }
                
                // check if band is activated and SWR was ok during previous run
                // if conditions are not met, roll-over
                do {
                  band_pointer = (band_pointer == (BAND_COUNT - 1)) ? 0 : band_pointer + 1;
                } while (!band_status[band_pointer] || !swr[band_pointer]);

                // generate band information to be displayed on LCD
                uint8_t offset = 0;
                if(band_pointer > 4) { offset = 1; }
                for(uint8_t i=0; i<3; i++) {
                  band[i] = pgm_read_byte_near(BAND_INFO + 3*band_pointer + offset + i);
                }

                deltaphase = deltaphase_base[band_pointer];
                if(beacon_mode) {
                  // set a fixed transmit frequency
                  deltaphase += dds.calculatePhaseValue(beacon_mode);
                }
                else {
                  // set a random transmit frequency
                  deltaphase += random(0, DELTAPHASE_BANDWITH);
                }

                on_air = 1;
              }
            }

            scheduleTask(0, 1); // Re-schedule in 100ms
          break;
// execute task 1 -> set new seed for the random number generator
          case 1:
            randomSeed(now());
            scheduleTask(1, 36000); // Re-schedule in 1h
          break;
// execute task 2 -> update those display items that require high-frequency refreshing
          case 2:
            switch(fast_refresh_pointer) {
              // refresh time & date
              case 0:
                static time_t ts_last = 0;
                time_t ts;
                ts = now();
                if(ts!=ts_last) {
                  ts_last = ts;
          
                  writeNumber(hour(ts), 20, 1, 2);
                  writeNumber(minute(ts), 23, 1, 2);
                  writeNumber(second(ts), 26, 1, 2);

                  uint8_t wd = 3*(weekday(ts)-1);
                  for(uint8_t i=0; i<3; i++) {
                    lcd_content[i + 2] = pgm_read_byte_near(WEEKDAY_CODING + i + wd);
                  }
          
                  writeNumber(day(ts), 4, 1, 2);
                  writeNumber(month(ts), 7, 1, 2);
                  writeNumber(year(ts) - 2000, 10, 1, 2);

                  loadLCD();
                }
              break;
              // calculate and display average SWR
              case 1:
                // calculate swr-average
                static uint16_t swr_average;
                swr_average = 0;
                for(uint8_t i=0; i<SWR_AVG_LENGTH; i++) {
                  swr_average += swr_avg[i];
                }
                swr_average /= SWR_AVG_LENGTH;

                lcd_content[27] = '0'+swr_average/10;
                lcd_content[29] = '0'+swr_average%10;

                loadLCD();
                break;        
            }
            
            scheduleTask(2, 1); // Re-schedule in 100ms
          break;
// execute task 3 -> not in use
          case 3:
            
          break;
// execute task 4 -> reschedule task 5
          case 4:
            scheduleTask(4, 35);
            scheduleTask(5, 0);
          break;
// execute task 5 -> change display content
          case 5:
            if(disp_content_pointer < 5) { setDisplayContent(); }
          break;
        }
      }
    }
    
    single_task_scheduler_loop_counter = 0;
  }
  
// increment all loop counters
  transmit_symbol_loop_counter++;
  system_timer_loop_counter++;
  single_task_scheduler_loop_counter++;
  check_SWR_loop_counter++;

// loop delay handling
// insures that on average the desired loop runtime will be met
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

// initializes an array of the type uint8_t with the same number
void initArray(uint8_t *target, uint8_t target_size, uint8_t value) {
  for(uint8_t i=0; i<target_size; i++) {
    *(target+i) = value;
  }
}

//##########################################################################################################

// calculate and returns the corresponding y to a given x based on a linear polynomial fit / returns 0 if x
// yields no result
// "x" -> input value
// "size" -> reference array size
// array_x & array_y -> reference arrays

uint16_t getPolyValue(const int16_t x, const uint16_t size, const uint16_t* array_x, const uint16_t* array_y) {

  uint16_t y = 0;
  uint16_t xbegin, xend, ybegin, yend;

// search for the supporting points enclosing x and if found, calculate y
  for(uint16_t i=0; i<size-1; i++) {
  
    xbegin = pgm_read_dword_near(array_x + i);
    xend = pgm_read_dword_near(array_x + i + 1);

    if((xbegin<=xend && x>=xbegin && x<=xend) || (xbegin>xend && x<=xbegin && x>=xend)) {

    ybegin = pgm_read_dword_near(array_y + i);
    yend = pgm_read_dword_near(array_y + i + 1);

      y = ybegin + int32_t((yend-ybegin)*(x-xbegin))/(xend-xbegin);
      break;
    }
  }

  return y;
}

//##########################################################################################################
// functions used for backlight control
//##########################################################################################################

// turns backlight pulsing on/off
void backlightPulsingOn(uint8_t pulsing) {
  if(pulsing) {
    if(!pulsing_on) {
      // initialize timer 2
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        brightness = 255;
        backlight_on_time_counter = 0;
        scheduleTask(4, -1);
        TCNT2  = 0; //initialize counter value to 0
        TCCR2B = 0b00000010; // prescaler 8
      }

      pulsing_on = 1;
    }
  }
  else {
    if(pulsing_on) {
      // disable timer 2
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR2B = 0; // set entire TCCR2B register to 0
        scheduleTask(4, 35);
      }
      
      digitalWrite(LED, 1);
      pulsing_on = 0;
    }
  }
}

//##########################################################################################################
// functions used for HD44780 control
//##########################################################################################################

// Sets the display content and updates the LCD; if short cycle updating is needed, a rapid update task will
// be started automatically
void setDisplayContent() {

// unschedule rapid updates  
  scheduleTask(2, -1);
  switch(disp_content_pointer) {
// display transmitter status
    case 0:
      if(!pulsing_on) {
        writeToBuffer(TX_ON_AIR);
        writeCharArray(band, 16, 3);
        if(SWR_METER_INSTALLED) {
          // slow down content switching when SWR is being displayed
          scheduleTask(4, 140);
          fast_refresh_pointer = 1;
          scheduleTask(2, 0);
        }
      }
      else {
        if(td) {
          writeToBuffer(TX_HARDWARE_DISABLED);
        }
        else {
          if(!tv) {
            writeToBuffer(TX_TIME_DISABLED);
          }
          else {
            if(!swr_sum) {
              writeToBuffer(TX_SWR_DISABLED);
              lcd_content[25] = 32;
              lcd_content[26] = 32;
              lcd_content[31] = 32;
            }
            else {
              if(swr[band_pointer]>30) {
                writeToBuffer(TX_SWR_DISABLED);
                writeCharArray(band, 28, 3);
              }
              else {
                writeToBuffer(TX_STANDBY);
              }
            }
          }
        }
      }
    break; 
// display last time synchronisation info
    case 1:
      if(last_sync) {
        writeToBuffer(TIME_LAST_SYNC);

        writeNumber(hour(last_sync), 13, 1, 2);
        writeNumber(minute(last_sync), 16, 1, 2);
        writeNumber(day(last_sync), 20, 1, 2);
        writeNumber(month(last_sync), 23, 1, 2);
        writeNumber(year(last_sync) - 2000, 26, 1, 2);
      }
      else {
        // LCD_TIME_NOT_SYNC_ADDRESS
        writeToBuffer(TIME_NOT_SYNC);
      }
    break;
 // display time, date & temp
    case 2:
// write fixed elements
      // LCD_TIME_FIXED_ADDRESS
      writeToBuffer(TIME_DATE);
// schedule rapid update of time & date
     if(last_sync) {
       fast_refresh_pointer = 0;
       scheduleTask(2, 0);
     }
    break;
  }

// increment the "display content pointer"
  disp_content_pointer = (disp_content_pointer == MAX_DISP_CONTENT) ? 0 : disp_content_pointer + 1;
  
  loadLCD();
}

//##########################################################################################################

// Updates the LCD content (only values that have changed will be transferred)
void loadLCD() {
  uint8_t c_pos = cursor_position;
  uint8_t c_stat = cursor_status;
  
  for(uint8_t i=0; i<DISPLAY_SIZE; i++) {
    if(lcd_content_last[i] != lcd_content[i]) {
      placeCursor(i, 0);
      lcd.write(lcd_content[i]);
      lcd_content_last[i] = lcd_content[i];
    }
  }
  
  placeCursor(c_pos, c_stat);
}

//##########################################################################################################

// Sets the cursor to a given position (0...31)
// with a given status (0==nothing; 1==only corsor; 2==only blinking; 3==blinking cursor)
void placeCursor(uint8_t c_pos, uint8_t c_stat) {
  cursor_position = c_pos;
  cursor_status = c_stat;
  
  uint8_t row = c_pos>>4;
  if(c_pos > 15) {
    c_pos -= 16;
  }
  lcd.setCursor(c_pos, row);

  switch(c_stat) {
    case 0:
      lcd.noCursor();
      lcd.noBlink();
    break;
    case 1:
      lcd.cursor();
      lcd.noBlink();
    break;
    case 2:
      lcd.noCursor();
      lcd.blink();
    break;
    case 3:
      lcd.cursor();
      lcd.blink();
    break;
    default:
      lcd.noCursor();
      lcd.noBlink();
  }
}

//##########################################################################################################

// Writes a full set of 32 characters from an array to the LCD screen buffer
void writeToBuffer(const uint8_t* array) {
  for(uint8_t i=0; i<32; i++) {
    lcd_content[i] = pgm_read_byte_near(array + i);
  }
}

//##########################################################################################################

// Writes a 4 digit number into consecutive LCD-segments; the number of leading zeros can be specified
// spanning from 0 to 3; the max. number of digits can be specified spanning from 1 to 4; neg. values will be
// preceeded by "-"
void writeNumber(int16_t number, int8_t pos, uint8_t leading_zeros, uint8_t max_digits) {
  char text[6] = " 0000";
  if(number < 0) {
    number = abs(number);
    text[0] = '-';
  }

  uint16_t dummy;
  uint8_t e = 1;
  int8_t sp = 1;
  uint8_t ns = 1;
  for(uint16_t i=1000; i>1; i/=10) {
    dummy = number/i;
    if(dummy) {
      text[e] += dummy;
      ns = 0;
    }
    else {
      if(ns) { sp++; }
    }
    number %= i;
    e++;
  }
  text[4] += number;
  
  sp -= leading_zeros;
  if(sp < 1) { sp = 1; }
  
  uint8_t ssize = 5-max_digits;
  if(ssize > sp) { sp = ssize; }
  
  if(text[0]=='-') {
    sp--;
    text[sp] = '-';
  }

  writeCharArray(&text[sp], pos+sp, 5-sp);
}

//##########################################################################################################

// Writes a set of characters to LCD screen buffer
void writeCharArray(const char* char_array, uint8_t pos, uint8_t len) {
  for(uint8_t i=0; i<len; i++) {
    lcd_content[i + pos] = *(char_array + i);
  }
}

//##########################################################################################################
// functions used for AD9850 control
//##########################################################################################################

// sets phase and restarts DDS
// if shutdown is 1, DDS will shut down, reducing the dissipated power from 380mW to 30mW @5V
void setPhaseValue(uint32_t deltaphase, uint8_t shutdown) {
  dds.setPhase(deltaphase, 0, shutdown);
	
  if(deltaphase) {
    backlightPulsingOn(0);
  }
  else {
    SWR_check_active = 0;
    symbol_counter = 0;
    on_air = 0;
    backlightPulsingOn(1);
    disp_content_pointer = 0;
    setDisplayContent();
  }

}

//##########################################################################################################
// functions used for task scheduling
//##########################################################################################################

// schedules a new task for single execution or unschedules a task
// "delay" should be given in 100ms resolution (10 == 1s; 0 == immediate execution; <0 == off/unschedule)

void scheduleTask(uint8_t id, int32_t delay) {
// check validity of task-id
  if(id <= MID) {

// schedule task by entering delay time into task-list
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if(delay > -1) {
        tl[id] = timer + delay;
      }
      else {
        tl[id] = -1;
      }
    }
  }
}
