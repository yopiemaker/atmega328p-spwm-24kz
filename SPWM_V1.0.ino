/*
 * PROJECT: ATmega328P SPWM Generator
 * AUTHOR: Yopie DIY
 * DATE: 15 feb 2026
 * VERSION: 1.0
 *
 * Copyright (C) 2024 Yopie DIY
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org>.
 * 
 * ============================================================================
 *                                DISCLAIMER
 * ============================================================================
 * WARNING: SPWM (Sinusoidal Pulse Width Modulation) implementation involves 
 * power electronics and potentially high-voltage circuitry. 
 * 
 * 1. This software is provided "as is" without any safety guarantees.
 * 2. The author is NOT responsible for any hardware damage (MOSFETs, 
 *    Microcontrollers, etc.) or personal injury resulting from its use.
 * 3. Ensure proper dead-time insertion and isolation (optocouplers) are 
 *    implemented in your hardware before testing.
 * 
 * USE AT YOUR OWN RISK.
 * ============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <analogComp.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// 
const int fMod = 50,            // AC sine frequency (Hz)
          fCarr = 24000;        // Carrier frequency (Hz)

const byte vfbPin = A0,
           tfbPin = A1,
           battPin = A2,
           F = A3,
           J2 = 12,
           buttonDownPin = 5,   //PD5      
           buttonUpPin = 4,     //PD4
           buttonSetPin = 2,    //PD2
           buttonPin = 52,      //0b00xx0x00 --> button pin mapped to port D
           buzzerCtl = 13,      //PB5 Buzzer and LED
           fanCtl = 8,          //PB0
           sPWM1 = 9,           //PB1
           sPwm2 = 10,          //PB2
           fund1 = 11,          //PB3
           fund2 = 3,           //PD3
           buttonDown = 36,     //0b0001 0100   PORTD = 0b0xx01x1xx    
           buttonUp = 20,       //0b0010 0100   PORTD = 0b0xx10x1xx 
           buttonSet = 48,      //0b0011 0000   PORTD = 0b0xx11x0xx 
           buttonSetUp = 16;    //0b0010 0000   PORTD = 0b0xx10x0xx --> button SET and UP pressed simultaneously

//  INVERTER PARAMETER
volatile int fanOnTemp=800,         // fan ON @45C --> 725
          fanOffTemp =679,          // fan OFF @40C --> 679
          fanOverTemp=924,          // over temp @ 80C --> 10k/(10k + 1.068k) x 1023 =924
          underVoltage = 462,       // underVoltage @ 2.5V --> 2.5 / 5 x 1023 = 512
          outputVoltage = 612,      // outputVoltage @ 3.0V --> 3.0 / 5 x 1023 = 612
          overVoltage = 645,        // overVoltage @ 3.15V --> 3.15 / 5 x 1023 = 645
          lowBatt = 200,            // if batt <= 11 V then switch off
          fullBatt = 550,           // if batt full cahrged then back to inverter mode
          systemBatt = 12;          // 

const int icr = (16000000/fCarr)/2,
          period = 1000000/fCarr,
          samples= (fCarr/fMod)/2;
          
const float deg=180.0/samples;
              
volatile float idxMod = 0.1;
volatile int idx;
volatile int phs;
volatile byte alarmOC;

int LUT[samples];

void initLUT() {
  int sineVal;
  for (int i=0; i<samples; i++){
    sineVal = int(sin(radians(deg*i))*icr + 0.5);
    LUT[i] = sineVal;
//    Serial.print(i);
//    Serial.print("=");
//    Serial.println(sineVal);
  }
}

void initTimer(){
  // Register initilisation, see datasheet for more detail.
  TCCR1A = 0b10110000;
                          //      10xxxxxx Clear OC1A/OC1B on compare match when up-counting. Set OC1A/OC1B on compare match when down counting
                          //      xx11xxxx Set OC1A/OC1B on compare match when up-counting. Clear OC1A/OC1B on compare match when down counting.
                          //      xxxxxx00 WGM1 1:0 for waveform 8 (phase freq. correct).
  TCCR1B = 0b00010001;
                          //      000xxxxx
                          //      xxx10xxx WGM1 3:2 for waveform mode 8.
                          //      xxxxx001 no prescale on the counter.
  TIMSK1 = 0b00000001;    //      xxxxxxx1 TOV1 Flag interrupt enable. 
  ICR1 = icr;             //      Counter TOP value (at 16MHz XTAL, SPWM carrier freq. 10kHz, 200 samples/cycle).
  sei();                  //      Enable global interrupts.
  DDRB = 0b10011110;               //      Disable Output
  PORTB = 0;
}

void initPort(){
  pinMode(fanCtl, OUTPUT);
  pinMode(buzzerCtl, OUTPUT);
  pinMode(buttonSetPin, INPUT_PULLUP);     
  pinMode(buttonUpPin, INPUT_PULLUP);           
  pinMode(buttonDownPin, INPUT_PULLUP);
  pinMode(fund1, OUTPUT);
  pinMode(fund2, OUTPUT);
  pinMode(AIN0, INPUT);
  pinMode(AIN1, INPUT);
  pinMode(J2, INPUT_PULLUP);
  pinMode(F,INPUT_PULLUP);
}

void welcomeScreen(){
  lcd.init();  // initialize the lcd
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Yopie DIY");  // Welcome screen
  lcd.setCursor(0, 1);
  lcd.print(" ATmega328 SPWM ");
}

void softStart(){
  idxMod = 0.01;
  for (int i = 0; i < 70; i++) {  // Soft Start
    idxMod = idxMod + 0.01;
    delay(40);
  }  
}

void uploadEEPROM(){
  if (EEPROM.read(0) == 0) {                                // EEPROM.read() = 255 if never used before
    lowBatt = (EEPROM.read(2)<<8) + (EEPROM.read(1));       // parameters int type, int = 2 byte = 16 bit EEPROM byte type
    underVoltage = (EEPROM.read(4)<<8) + (EEPROM.read(3));  // EEPROM byte type, byte = 8 bit
    overVoltage = (EEPROM.read(6)<<8) + (EEPROM.read(5));   // so one parameters value need two EEPROM value
    fanOverTemp = (EEPROM.read(8)<<8) + (EEPROM.read(7));
  }  
}

void initAnalogComp(){
  analogComparator.setOn(AIN0, AIN1); //we instruct the lib to use voltages on the pins
  analogComparator.enableInterrupt(interruptOverCurrent,RISING);
  alarmOC = 0;
}

void interruptOverCurrent() {
    TCCR1A = 0;
    alarmOC = 1;
}

void alarmIndication(int alarm) {
  TCCR1A = 0;  // shutdown SPWM output
  TIMSK1 = 0;
  PORTB &= 0b11100001;
  PORTD &= 0b11110111;

  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WARNING!");
  lcd.setCursor(0, 1);
  if (alarm == 2) lcd.print(" UNDER VOLTAGE");
  if (alarm == 3) lcd.print("  OVER VOLTAGE");
  if (alarm == 4) lcd.print("OVER TEMPERATURE");
  if (alarm == 5) lcd.print("  LOW BATTERY");
  if (alarm == 6) lcd.print(" SHORT CIRCUIT");
  if (alarm == 7) lcd.print("  OVER CURRENT ");

  while (1) {                         // run until reset
    for (int i = 0; i < alarm; i++) {
      digitalWrite(buzzerCtl, HIGH);  // turn ON LED and Buzzer
      delay(200);
      digitalWrite(buzzerCtl, LOW);  // then turn OFF
      delay(200);
    }
  delay(1000);
  }
}

void feedBackTest(float vfbIn, int tfbIn, int battIn) {
  long alrm;
  static int alrmCnt;
  static int dispCnt;
  float dis1;
  float dis2;
  float dis3;

  if (digitalRead(J2) == LOW) goto noFeedBackTest;
  if (phs != 1) return;

  alrm = constrain(vfbIn, underVoltage, overVoltage);
  if (alrm != vfbIn) alrmCnt++;  
  else    
    alrmCnt = 0;
  if (alrm == underVoltage && alrmCnt >= 150) alarmIndication(2);               // underVoltage @ 2.5V --> 2.75 / 5 x 1023 = 562
  if (alrm == overVoltage && alrmCnt >= 15) alarmIndication(3);                 // overVoltage @ 3.15V --> 3.15 / 5 x 1023 = 645
  if (tfbIn >= fanOverTemp) alarmIndication(4);                                 // over temp @ 80C --> 10k/(10k + 1.068k) x 1023 =924
  if (tfbIn >= fanOnTemp && digitalRead(8) == LOW) digitalWrite(fanCtl, HIGH);  // fan ON @45C --> 725
  if (tfbIn <= fanOffTemp && digitalRead(8) == HIGH) digitalWrite(fanCtl, LOW); // fan OFF @40C --> 679
//  if (battIn <= lowBatt && digitalRead(F) == HIGH) alarmIndication(5);          // low batt @ 10.5V --> (2k7/12.7k x 10.5) / 5 x 1023 = 457

noFeedBackTest:
  if (dispCnt >= 50) {        // display updated every 50 cycle to avoid flickering
    dis1 = battIn * 0.02299*2.1;  // constant is the result of reversing the above calculation
    dis2 = vfbIn * 0.3560;
    dis3 = ((tfbIn - 512) / 11.0) + 25.0;
    lcd.setCursor(0, 1);
    lcd.print(String(dis1, 1) + "  " + String(dis2, 0) + "   " + String(dis3, 1));
    dispCnt = 0;
  }
  dispCnt++;
}

//-------------------------------- Function parameter editing -----------------------------------
void editParameters(){
  int lowBattTmp = lowBatt,             // copy value to temporary variable
      underVoltageTmp = underVoltage, 
      overVoltageTmp = overVoltage, 
      fanOverTempTmp = fanOverTemp;

  lcd.cursor();
  lcd.blink();

//---------------------------------------- BATTERY LOW --------------------------------------------
  lcd.clear();  
  lcd.print("  Low Battery: "); 
  lcd.setCursor(6, 1); 
  lcd.print(float(lowBatt/512.0*systemBatt),1);

  while ((PIND & buttonPin) != buttonPin) delay(1);   // wait for key released (triggered in loop)
  do {
    if ((PIND & buttonPin) == buttonUp) {              // key UP
      lowBattTmp = lowBattTmp +3;
      lcd.setCursor(6,1);  lcd.print(float(lowBattTmp/512.0*systemBatt),1);  
    } 
    while ((PIND & buttonPin) == buttonUp) delay(1);   // wait until UP key release

    if ((PIND & buttonPin) == buttonDown) {              // key DOWN
      lowBattTmp = lowBattTmp -3;
      lcd.setCursor(6, 1);  lcd.print(float(lowBattTmp/512.0*systemBatt),1); 
    } 
    while ((PIND & buttonPin) == buttonDown) delay(1);  // wait until DOWN key release
  } while ((PIND & buttonPin) != buttonSet);           // do "this loop" while "the key is not SET key"
  while ((PIND & buttonPin) == buttonSet) delay(1);   // SET key pressed. wait until released

//---------------------------------------- UNDER VOLTAGE -------------------------------------------
  lcd.clear();
  lcd.print(" Under Voltage: "); 
  lcd.setCursor(5, 1);
  lcd.print(float(underVoltageTmp * 0.3584),1); 
  do {
    if ((PIND & buttonPin) == buttonUp) {              // key UP
      underVoltageTmp = underVoltageTmp + 3;
      lcd.setCursor(5,1);  lcd.print(float(underVoltageTmp * 0.3584),1);  
    }  
    while ((PIND & buttonPin) == buttonUp) delay(1);   // wait until UP key release

    if ((PIND & buttonPin) == buttonDown) {              // key DOWN
      underVoltageTmp = underVoltageTmp - 3;
      lcd.setCursor(5, 1);  lcd.print(float(underVoltageTmp * 0.3584),1); 
    }
    while ((PIND & buttonPin) == buttonDown) delay(1);  // wait until DOWN key release
  }  while ((PIND & buttonPin) != buttonSet);           // do "this loop" while "the key is no SET key"
  while ((PIND & buttonPin) == buttonSet) delay(1);   // SET key pressed. wait until released 

//---------------------------------------- OVER VOLTAGE -----------------------------------------------
  lcd.clear();
  lcd.print("  Over Voltage: "); 
  lcd.setCursor(5, 1);
  lcd.print(float(overVoltageTmp*0.3584),1); 
  do {
    if ((PIND & buttonPin) == buttonUp) {              // key UP
      overVoltageTmp = overVoltageTmp + 3;
      lcd.setCursor(5,1);  lcd.print(float(overVoltageTmp*0.3584),1);  
    }  
    while ((PIND & buttonPin) == buttonUp) delay(1);   // wait until UP key release

    if ((PIND & buttonPin) == buttonDown) {              // key DOWN
      overVoltageTmp = overVoltageTmp - 3;
      lcd.setCursor(5, 1);  lcd.print(float(overVoltageTmp*0.3584),1); 
    }
    while ((PIND & buttonPin) == buttonDown) delay(1);  // wait until DOWN key release
  }  while ((PIND & buttonPin) != buttonSet);           // do "this loop" while "the key is no SET key"
  while ((PIND & buttonPin) == buttonSet) delay(1);   // SET key pressed. wait until released 

//---------------------------------------- OVER TEMPERATURE -------------------------------------------
  lcd.clear();
  lcd.print("   Over Temp: "); 
  lcd.setCursor(5, 1);
  lcd.print(float(((fanOverTempTmp - 512) / 11.0) + 25.0),1); 
  do {
      if ((PIND & buttonPin) == buttonUp) {              // key UP
      fanOverTempTmp = fanOverTempTmp + 9;
      lcd.setCursor(5,1);  lcd.print(float(((fanOverTempTmp - 512) / 11.0) + 25.0),1);  
    }  
    while ((PIND & buttonPin) == buttonUp) delay(1);   // wait until UP key release

    if ((PIND & buttonPin) == buttonDown) {              // key DOWN
      fanOverTempTmp = fanOverTempTmp - 9;
      lcd.setCursor(5, 1);  lcd.print(float(((fanOverTempTmp - 512) / 11.0) + 25.0),1); 
    }
    while ((PIND & buttonPin) == buttonDown) delay(1);  // wait until DOWN key release
  }  while ((PIND & buttonPin) != buttonSet);           // do "this loop" while "the key is no SET key"
  while ((PIND & buttonPin) == buttonSet) delay(1);   // SET key pressed. wait until released 

  lcd.clear();
  lcd.print("  SET = UPDATE  ");
  lcd.setCursor(0, 1); 
  lcd.print("UP/DOWN = CANCEL");
  lcd.setCursor(15, 0);

  while ((PIND & buttonPin) == buttonPin) delay(1); // wait until Save or Cancel key pressed 

//---------------------------------------- SAVE / CANCEL ---------------------------------------
  if ((PIND & buttonPin) == buttonSet) {            // if Save then do this, cancel then skip this section
    lowBatt = lowBattTmp;
    underVoltage = underVoltageTmp; 
    overVoltage = overVoltageTmp;
    fanOverTemp = fanOverTempTmp;
    EEPROM.update(0, 0);  
    EEPROM.update(1, lowByte(lowBatt));             // lowBatt and etc are integer (2byte) and EEPROM is in bytes 
    EEPROM.update(2, highByte(lowBatt));            // do a split between highByte and lowByte
    EEPROM.update(3, lowByte(underVoltage));        
    EEPROM.update(4, highByte(underVoltage));
    EEPROM.update(5, lowByte(overVoltage));
    EEPROM.update(6, highByte(overVoltage));
    EEPROM.update(7, lowByte(fanOverTemp));
    EEPROM.update(8, highByte(fanOverTemp));
    lcd.clear();
    lcd.println("    UPDATED       ");              // update done
    delay(1000);
  }
  while ((PIND & buttonPin) != buttonPin) delay(1); // wait until Save or Cancel key released 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Batt--Vout--Temp");

}

void setup() {
  Serial.begin(115200); 
  welcomeScreen();  
  initLUT();
  initAnalogComp(); 
  uploadEEPROM();    
  initTimer();
  initPort();
  softStart();
  alarmOC=0;
  lcd.clear(); 
  lcd.setCursor(0, 0); 
  lcd.print("Batt--Vout--Temp");
}

void loop() {
 
  static int vfbValue;
  static int vfbValue1;
  static int vfbRise = 0;
  static int vMax;
  float alrm = 0;
  static int alrmCnt = 0;
  float ampDiff;
  static unsigned long arduinoTime, buttonTime;


  if ((PIND & buttonPin) == buttonSetUp) {                // check for SET and UP buttons pressed together
    buttonTime = millis() - arduinoTime;                  // check how long the key pressed
    if (buttonTime > 2000)  editParameters();             // if longer than 2 sec then call edit function 
  } else arduinoTime=millis();                            // save the time value when the button is NOT PRESSED 


  if (phs == 1) {
    vfbValue1 = vfbValue;
    vMax=max(vMax,vfbValue);

    vfbValue = analogRead(vfbPin);  // vfbPin = 0 to 5V ---> vfbValue = 0 to 1023

    if (vfbValue > vfbValue1 && vfbValue > 300) vfbRise = 1;  //check for positif cycle, bigger than noise

    if (vfbValue < vfbValue1 && vfbRise == 1) {               // maximum vfb value reached
      vfbRise = 0;
      ampDiff = 614 - vfbValue1;
      if (ampDiff > 5 || ampDiff < -5) {
        idxMod = idxMod + (ampDiff / vfbValue1);  // voltage correction
        if (idxMod > 0.97) idxMod = 0.97;    // limit index modulation to 97%
      }
      feedBackTest(vfbValue1, analogRead(tfbPin), analogRead(battPin));    
    }
    
    if (vMax<=50 && phs==0) alrmCnt++;     // Vout<=18V alarm (50/614x220V <= 18V)
    if (alrmCnt>=1 && digitalRead(J2)==HIGH) alarmIndication(6);   // when occur for 1 cycle or 100mS
    if (alarmOC==1 && digitalRead(J2)==HIGH) alarmIndication(7);
  } else  vMax=0;
}

/*---------------------------------------------------------------------------------------------------------*/
ISR(TIMER1_OVF_vect) {
//  static int idx;
  static int ph;
  static int dtA; //= 0;
  static int dtB; //= 5;

//  PORTB |= (1 << 0);  // debug pin (FanCTR) --> rising

  if (idx == 0) {   
    if (ph == 0) {          // OC1A as SPWM out
      TCCR1A = 0b10110000;  // clear OC1A, set OC1B on compare match
      dtA = 0;              // no dead time
      dtB = 5;              // adding dead time to OC1B
    } else {
      TCCR1A = 0b11100000;  // OC1B as SPWM out
      dtA = 5;
      dtB = 0;
    }
    PORTB &= 0b11110111;    // 1HO & 1LO Off
    PORTD &= 0b11110111;    
    ph ^= 1;
  }
  if (idx == 1) {  
    if (ph == 1) {
      PORTB &= 0b11110111;      // fund2 Off
      asm volatile ("nop");
      asm volatile ("nop");
      PORTD |= 0b00001000;      // fund1 On
      phs = 1;
    } else {
      PORTD &= 0b11110111;      // fund1 Off
      asm volatile ("nop");
      asm volatile ("nop");
      PORTB |= 0b00001000;      // fund2 On
      phs = 0;
    }
  }


  idx++;
  int OCR_val = int(LUT[idx] * idxMod);   // OCR1x value for next update
  OCR1A = OCR_val + dtA;
  OCR1B = OCR_val + dtB;

  if (idx >= samples) idx=0;

//  PORTB &= ~(1 << 0); // debug pin (FanCTR) --> falling
}

