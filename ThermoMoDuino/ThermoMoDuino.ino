/*
 * thermoMoDuino The Thermo-Module-Arduino implementation
 * Copyright (C) 2020 D.Herrendoerfer 
 *                    <d.herrendoerfer@herrendoerfer.name>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/
 
/*
 * This is a thermostat with a cooling algorithm for a chiller with
 *  1. A compressor
 *  2. A bypass valve and solenoid
 *  3. A cooler fan
 *  
 *  The basic algorithm features the following states:
 *  
 *  N   Name         State                                  Go to net when         Bail out if and to
 *  0: init                                                 everything ok
 *  1: Idle          Nothing on:                            reach T+hysteresis     ---
 *  2: Starting      bypass on, fan on,                     timer of 5s            ---
 *  3: Cooling       compressor on, bypass off, fan on      reach T                ---
 *  4: Holding       compressor on, bypass on, fan on       timer 600s             reach T -> 3
 *  5: Cooldown      compressor off, bypass off, fan on     timer 60s              reach T -> 2
 */

#include <Arduino.h>
#include <TM1637Display.h>

// LCD module connection pins (digital pins)
#define CLK 2
#define DIO 3

// thermistors
#define ThermistorPin  0
#define Thermistor2Pin 1

// relays
#define SolenoidPin 7
#define CompressorPin 8
#define FanPin 10
#define PumpPin 9

// cooling temp settings
float hysteresis = 0.8;
float targetTemp = 16.0;
float targetCaseTemp = 22.0;

// case temp management (inside the chiller)
#define INTELLI_CASE 1
#define CASE_TEMP_MARGIN 2.0
#define CASE_TEMP_ROOF 30

// cooling time settings
#define HOLD_TIME  600     
#define COOLDOWN_TIME  60

// ********************************** INTERNALS *******************************
// temp reading
int Vo;
float R1 = 10000;
float logR2, R2, T, T2, Tlast, T2last;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

// display
TM1637Display display(CLK, DIO);

// time
#define SECOND 1000
unsigned long thisTick;
unsigned long lastTick;
unsigned long nextSecondTick;
unsigned long seconds;

//log
int logInterval = 1; // log every 1 to 9 seconds

#define T_MAX 50.0
#define T_MIN  3.0
#define T2_MAX 55.0
#define T2_MIN  3.0

// state machine
int state = 0;
int state_last = 0;
int state_new = 0;

//
// 7-Segment Text
const uint8_t SEG_ERR[] = {
  SEG_A | SEG_E | SEG_F | SEG_D | SEG_G,           // E
  SEG_E | SEG_G,                                   // r
  SEG_E | SEG_G,                                   // r
  0                                                // 
  };

const uint8_t SEG_INIT[] = {
  SEG_C ,                                          // i
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_C ,                                          // i
  0                                                // 
  };

const uint8_t SEG_IDLE[] = {
  SEG_E | SEG_F,                                   // I
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_D | SEG_E | SEG_F ,                          // L
  SEG_A | SEG_E | SEG_F | SEG_D | SEG_G            // E
  };

const uint8_t SEG_COOL[] = {
  SEG_D | SEG_E | SEG_F | SEG_A ,                  // C
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_D | SEG_E | SEG_F                            // L
  };

const uint8_t SEG_HOLD[] = {
  SEG_B | SEG_C | SEG_E | SEG_F | SEG_G ,          // H
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_D | SEG_E | SEG_F,                           // L
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G            // d
  };

const uint8_t SEG_FAN[] = {
  SEG_A | SEG_E | SEG_F | SEG_G ,                  // F
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,   // A
  SEG_C | SEG_E | SEG_G,                           // n
  0                                                // 
  };

const uint8_t SEG_DEGC[] = {
  SEG_A | SEG_B | SEG_G | SEG_F,                   // deg
  SEG_A | SEG_E | SEG_F | SEG_D                    // C
  };


void setup() 
{
  Serial.begin(9600);
  display.setBrightness(0x0f);

  pinMode(CompressorPin,OUTPUT);
  digitalWrite(CompressorPin,HIGH);
  pinMode(SolenoidPin,OUTPUT);
  digitalWrite(SolenoidPin,HIGH);
  pinMode(FanPin,OUTPUT);
  digitalWrite(FanPin,HIGH);
  pinMode(PumpPin,OUTPUT);
  digitalWrite(PumpPin,HIGH);
}

void read_temp()
{
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;

  if ( Tlast != 0)
    T=((Tlast*199.0)+T)/200.0;
  Tlast=T;

  Vo = analogRead(Thermistor2Pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T2 = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T2 = T2 - 273.15;

  if ( T2last != 0)
    T2=((T2last*99.0)+T2)/100.0;
  T2last=T2;
}

void compressor(int state)
{
  if (state)
    digitalWrite(CompressorPin, LOW);  
  else
    digitalWrite(CompressorPin, HIGH);
}

void solenoid(int state)
{
  if (state)
    digitalWrite(SolenoidPin, LOW);  
  else
    digitalWrite(SolenoidPin, HIGH);
}

void fan(int state)
{
  if (state)
    digitalWrite(FanPin, LOW);  
  else
    digitalWrite(FanPin, HIGH);
}

//State 1 variables

int do_state_1() 
{
  fan(0);
  compressor(0);
  solenoid(0);
  
  if (T2 > targetCaseTemp)
    state_new = 5;

  if (T > targetTemp+hysteresis)
    state_new = 2;
    
  return 0;  
}


//State 2 variables
unsigned long st2_timer = 0;

int do_state_2() 
{
  fan(0);
  compressor(1);
  solenoid(1);

  if (state_last != 2)
    st2_timer=seconds+3;

  if (seconds == st2_timer)
    state_new = 3;
    
  return 0;  
}

//State 3 variables
unsigned long st3_start = 0;

int do_state_3() 
{
  fan(1);
  compressor(1);
  solenoid(0);

  if (state_last != 3)
    st3_start=seconds;

  if (T <= targetTemp && seconds-st3_start > 5)
    state_new = 4;

  return 0;  
}

//State 4 variables
unsigned long st4_timer = 0;

int do_state_4() 
{
  fan(1);
  compressor(1);
  solenoid(1);

  if (state_last != 4)
    st4_timer=seconds+HOLD_TIME;

  if (seconds == st4_timer)
    state_new = 5;

  if (T > targetTemp + 0.2 && st4_timer-seconds < (HOLD_TIME - 5))
    state_new = 3;

  if (T < targetTemp - hysteresis)
    state_new = 5;

  return 0;  
}

//State 5 variables
unsigned long st5_timer = 0;

int do_state_5() 
{
  fan(1);
  compressor(0);
  solenoid(0);

  if (state_last != 5)
    st5_timer=seconds+COOLDOWN_TIME;

  if (seconds == st5_timer) {
    state_new = 1;

    #ifdef INTELLI_CASE
    targetCaseTemp = (((targetCaseTemp*3.0)+T2+CASE_TEMP_MARGIN)/4.0) ;
    if (targetCaseTemp > CASE_TEMP_ROOF)
      targetCaseTemp = CASE_TEMP_ROOF;
    #endif
  }

  if (T > targetTemp+hysteresis)
    state_new = 2;

  return 0;  
}

void errmesg(int err)
{
  fan(0);
  compressor(0);
  solenoid(0);

  while (true) {
    display.setSegments(SEG_ERR);
    tone(5,4500,1000);
    delay(1000);
    display.showNumberDec((int)err, false,3,0);
    delay(1000);
  }
}

void loop() 
{
  boolean tick;

  // Initialization
  display.setSegments(SEG_INIT);

  for ( int i=0; i<200; i++){
    read_temp();
    delay(20);
  }

  if (T<T_MIN ||T>T_MAX){
    errmesg(1);
  }
  if (T2<T2_MIN ||T2>T2_MAX){
    errmesg(2);
  }

  // Click the relays once (but not for the compressor)
  delay(1000);
  solenoid(1);
  delay(3000);
  solenoid(0);
  delay(1000);
  fan(1);
  delay(3000);
  fan(0);

  delay(3000);
  state=1;

  // Init the last tick
  lastTick=millis();
  nextSecondTick=millis()+SECOND;

  Serial.println("Water,Case,Target,Target+Hyst,CaseTarget,State");

  // Main duty loop
  while (true) {
    thisTick=millis();

    //Calculate second counter
    if (!(nextSecondTick < 1001 && thisTick > 1001)) {
      // Jump past the roll-over
      if (thisTick > nextSecondTick) {
        seconds++;
        nextSecondTick += SECOND;
        tick=true;
      }
      else {
        tick=false;
      }
    }
    else {
      tick=false;
    }

    // Read the sensors
    read_temp();

    // Cycle through the state machine once
    state_new = 0;
    
    if ( state == 1){
      if ( do_state_1())
        errmesg(11);      
    }
    else if ( state == 2){
      if ( do_state_2())
        errmesg(11);      
    }
    else if ( state == 3){
      if ( do_state_3())
        errmesg(11);      
    }
    else if ( state == 4){
      if ( do_state_4())
        errmesg(11);      
    }
    else if ( state == 5){
      if ( do_state_5())
        errmesg(11);      
    }
    else 
      errmesg(99);

    // If needed change the state
    state_last = state;
    if (state_new)
      state = state_new;

    //Safeguards
    if (T<T_MIN ||T>T_MAX){
      errmesg(1);
    }
    if (T2<T2_MIN ||T2>T2_MAX){
      errmesg(2);
    }

    // Logging and Display
    if (tick) {
      if (seconds % logInterval == 0) {
        Serial.print(T);
        Serial.print(",");
        Serial.print(T2);
        Serial.print(",");
        Serial.print(targetTemp);
        Serial.print(",");
        Serial.print(targetTemp+hysteresis);
        Serial.print(",");
        Serial.print(targetCaseTemp);
        Serial.print(",");
        Serial.print(state);
        Serial.println();
      }
      
      if (seconds % 5 != 0){
        if (state == 1) {
          display.setSegments(SEG_IDLE);
        } 
        else if (state == 3) {
          display.setSegments(SEG_COOL);
        } 
        else if (state == 4) {
          display.setSegments(SEG_HOLD);
        } 
        else if (state == 5) {
          display.setSegments(SEG_FAN);
        } 
      }
      else {
        display.showNumberDec((int)(T+0.5), false,2,0);
        display.setSegments(SEG_DEGC, 2, 2); 
      }

      if (state == 1 && seconds % 10 == 0)
        seconds=0;
    }  
   delay(50);
  }
}
