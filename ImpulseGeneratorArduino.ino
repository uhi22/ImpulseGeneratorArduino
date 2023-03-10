/* Impulse-Generator
 *  
 * Features:
 *   - create voltage ramps and pulses
 *   
 * Hardware:
 *   - Arduino Nano (Board in Arduino IDE: "Arduino Nano, ATmega328P")
 *   - DAC MCP4922 via SPI
 * 
 * Change Log:
 *  2022-09-28 Uwe: Added analog output via SPI-DAC
 *  2023-01-24 Uwe: positive pulses
 * 
 * Todos:
 * 
 * References:
 *   https://www.best-microcontroller-projects.com/mcp4922.html
 *   
 */

#include <SPI.h>

int RCLKPin  = 10;   // chip select (nSS)
int SRCLKPin = 13;   // clock - SCK
int SERPin   = 11;   //  MOSI

SPISettings settingsA(16000000, MSBFIRST, SPI_MODE0);  // At 16 = SPI Clock = 8MHz.

#define PULSE_PIN 8 /* pin where the output pulse is produced */

uint16_t tPulseHigh_ms = 5;
uint16_t tPulseLow_ms = 5;
uint8_t nPulseSection=0;
uint16_t nPulseSubTime=0;
uint16_t nHighTime_ms = 1;
uint16_t nCounterOfIdenticalPulses = 0;


void setup() {
  pinMode(PULSE_PIN,OUTPUT);
  digitalWrite(PULSE_PIN,LOW);

  pinMode(RCLKPin,  OUTPUT);   // Set SPI control PINs to output.
  pinMode(SRCLKPin, OUTPUT);
  pinMode(SERPin,   OUTPUT);

      SPI.begin();
}

//////////////////////////////////////////////////////////////////////////////
// 0 - A, 1 - B
//
void writeMCP4922_AB(byte AB, uint16_t v) {

    v |=0xf000;             // B15(A/B)=1 B, B14(BUF)=1 on, B13(GAn) 1=x1  B12(SHDNn) 1=off
    if (!AB)  v &= ~0x8000; // When zero clear B15 for A.

    SPI.beginTransaction(settingsA);
    digitalWrite(RCLKPin, LOW);
    SPI.transfer( (0xff00 & v)>>8 );
    SPI.transfer(      0x00ff & v );
    digitalWrite(RCLKPin, HIGH);
    SPI.endTransaction;
}



void pulseGenerator1(void) {
  uint16_t vOut;
  switch (nPulseSection) {
    case 0:
       vOut = 4095;
       nPulseSubTime++;
       if (nPulseSubTime>500) {
          nPulseSubTime=0;
          nPulseSection=1;
       }
       break;
    case 1:
       vOut = 4095-nPulseSubTime;
       nPulseSubTime++;
       if (nPulseSubTime>2000) {
          nPulseSubTime=0;
          nPulseSection=2;
       }
       break;
    case 2:
       vOut = 400;
       nPulseSubTime++;
       if (nPulseSubTime>500) {
          nPulseSubTime=0;
          nPulseSection=3;
       }
       break;
    case 3:
       vOut = 0;
       nPulseSubTime++;
       if (nPulseSubTime>50) {
          nPulseSubTime=0;
          nPulseSection=0;
       }
       break;
  }
  writeMCP4922_AB( 0, vOut );
}

/* create positive pulses. Width starts with 1ms and increases up to 500ms.
   Off-phase is fix 500ms. */
#define MAXIMUM_HIGH_TIME_MS 500
#define LOW_TIME_MS 500
void pulseGenerator2_positivePulses(void) {
  uint16_t vOut;
  switch (nPulseSection) {
    case 0:
       vOut = 4095; /* full ON */
       nPulseSubTime++;
       if (nPulseSubTime>=nHighTime_ms) {
          nPulseSubTime=0;
          nPulseSection=1;
       }
       break;
    case 1:
       vOut = 0; /* 0V */
       nPulseSubTime++;
       if (nPulseSubTime>=LOW_TIME_MS) {
          nPulseSubTime=0;
          nPulseSection=0;
          nCounterOfIdenticalPulses++;
          if (nCounterOfIdenticalPulses>=5) {
              nHighTime_ms++;
              nCounterOfIdenticalPulses = 0;
              if (nHighTime_ms>=MAXIMUM_HIGH_TIME_MS) { nHighTime_ms=1; }
          }
       }
       break;
  }
  writeMCP4922_AB( 0, vOut );
}

void demoDac(void) {
  int i;
  for (i=0; i<4096; i+=4) {
    writeMCP4922_AB( 0, i );
  }  
}

void loop() {
  //digitalWrite(PULSE_PIN,HIGH);
  //delay(tPulseHigh_ms);
  //digitalWrite(PULSE_PIN,LOW);
  //delay(tPulseLow_ms);
  //pulseGenerator1();
  pulseGenerator2_positivePulses();
  delay(1); /* wait 1ms */

}
