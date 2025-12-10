/**
 * Final Project
 * Group 2
 * Members:
 * - Humberto Cortez
 * - Mason Chacon
 * - Evan Barnett
 * - Riley Padilla
 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

// Pin Definitions

#define PIN_LED_BLUE   4    // PH4 D7. OUT
#define PIN_LED_GREEN  5    // PH5 D8. OUT
#define PIN_LED_YELLOW 6    // PH6 D9. OUT
#define PIN_LED_RED    4    // PB4 D10 OUT

#define PIN_BTN_START  4    // PE4 D2  IN
#define PIN_BTN_STOP   5    // PE5 D3  IN
#define PIN_BTN_CLEAR  5    // PG5 D4  IN

#define PIN_FAN 12          // PB6 D12
#define PIN_DHT11 22        // PA0 D22
#define PIN_WATER_SENSOR 0  // Analog 0, to be used with adcRead()
#define PIN_POTENTIOMETER 1 // Analog 1, to be used with adcRead()

#define PIN_CLK_SDA 20      // 
#define PIN_CLK_SCL 21      // 

#define PIN_DIS_RS 30       // D30
#define PIN_DIS_EN 31       // D31
#define PIN_DIS_D4 32       // D32
#define PIN_DIS_D5 33       // D33
#define PIN_DIS_D6 34       // D34
#define PIN_DIS_D7 35       // D35

#define STEPPER_STEPS 1024
#define PIN_STEPPER_IN1 42  // D42
#define PIN_STEPPER_IN2 43  // D43
#define PIN_STEPPER_IN3 44  // D44
#define PIN_STEPPER_IN4 45  // D45

// UART Pointers
volatile unsigned char* pUCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char* pUCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char* pUCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int*  pUBRR0  = (unsigned  int*) 0x00C4;
volatile unsigned char* pUDR0   = (unsigned char*) 0x00C6;
// ADC Pointers
volatile unsigned char* pADMUX   = (unsigned char*) 0x7C;
volatile unsigned char* pADCSRB  = (unsigned char*) 0x7B;
volatile unsigned char* pADCSRA  = (unsigned char*) 0x7A;
volatile unsigned int*  pADC_DATA = (unsigned int*) 0x78;
// GPIO Pointers
volatile unsigned char* pDDRB   = (unsigned char*) 0x24;
volatile unsigned char* pPORTB  = (unsigned char*) 0x25;
volatile unsigned char* pDDRH   = (unsigned char*) 0x101;
volatile unsigned char* pPORTH  = (unsigned char*) 0x102;

// Functions Declarations
void U0Init(int baud);
unsigned char U0KbHit();
unsigned char U0getChar();
void U0putChar(unsigned char ch);
void printString(const char* str);


// Main Program

LiquidCrystal lcd(PIN_DIS_RS, PIN_DIS_EN, PIN_DIS_D4, PIN_DIS_D5, PIN_DIS_D6, PIN_DIS_D7);
Stepper stepper(STEPPER_STEPS, PIN_STEPPER_IN1, PIN_STEPPER_IN3, PIN_STEPPER_IN2, PIN_STEPPER_IN4);

int stepperPrev = 0;

void setup(){
    U0Init(9600);
    adcInit();
    lcd.begin(16, 2);
    stepper.setSpeed(26);
    stepperPrev = adcRead(PIN_POTENTIOMETER);

    printString("--- Swap Cooler Final Project ---\n");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println("Final Project");
    lcd.println("Swap Cooler");
    delay(2000);
}

void loop(){
    int potValue = adcRead(PIN_POTENTIOMETER);
    int stepVal = potValue - stepperPrev;
    if(stepVal != 0) stepper.step(stepVal);
    stepperPrev = potValue;
}

// ADC Functions

void adcInit(){
  // setup the A register
  *pADCSRA |= 0b10000000; // set bit 7 to 1 to enable the ADC 
  *pADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *pADCSRA &= 0b11110111; // clear bit 3 to 0 to disable the ADC interrupt 
  *pADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *pADCSRA |= 0b00000111; // set bit 0-2 to 111 to set prescaler selection to 128 for 16MHz/128=125KHz
  
  // setup the B register
  *pADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *pADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  
  // setup the MUX Register
  *pADMUX &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *pADMUX |= 0b01000000; // set bit 6 to 1 for AVCC analog reference
  *pADMUX &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *pADMUX &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adcRead(unsigned char channel){
  *pADMUX &= 0b11100000; // clear the channel selection bits (MUX 4:0)
  *pADCSRB &= 0b11110111; // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register 
  *pADMUX |= (channel & 0x1F); // set the channel selection bits for channel
  *pADCSRA |= 0b01000000; // set bit 6 of ADCSRA to 1 to start a conversion

  while((*pADCSRA & 0x40) != 0); // wait for the conversion to complete
  
  return (*pADC_DATA & 0x03FF); // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
}


// UART Functions

void U0Init(int baud){
    unsigned long FCPU = 16000000;
    unsigned int tbaud = (FCPU / 16 / baud - 1);

    *pUCSR0A = 0x20;
    *pUCSR0B = 0x18;
    *pUCSR0C = 0x06;
    *pUBRR0  = tbaud;
}

unsigned char U0KbHit(){
    return *pUCSR0A & 0x80;
}

unsigned char U0getChar(){
    return *pUDR0;
    
}
void U0putChar(unsigned char ch){
    while((*pUCSR0A & 0x20) == 0);
    *pUDR0 = ch;
}

void printString(const char* str){
    int i = 0;
    while(str[i] != '\0'){
        U0putChar(str[i]);
        i++;
    }
}