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

// Pin Definitions

#define PIN_LED_BLUE   4       // PH4 D7
#define PIN_LED_GREEN  5       // PH5 D8
#define PIN_LED_YELLOW 6       // PH6 D9
#define PIN_LED_RED    4       // PB4 D10

#define PIN_BTN_START  4       // PE4 D2
#define PIN_BTN_STOP   5       // PE5 D3
#define PIN_BTN_CLEAR  5       // PG5 D4

#define PIN_FAN 12      // PB6 D12
#define PIN_DHT11 22 // PA0 D22
#define PIN_WATER_SENSOR A0 //
#define PIN_POTENTIOMETER A1

#define PIN_CLK_SDA 20
#define PIN_CLK_SCL 21

#define PIN_DIS_RS 30
#define PIN_DIS_EN 31
#define PIN_DIS_D4 32
#define PIN_DIS_D5 33
#define PIN_DIS_D6 34
#define PIN_DIS_D7 35

#define PIN_STEPPER_IN1 42
#define PIN_STEPPER_IN2 43
#define PIN_STEPPER_IN3 44
#define PIN_STEPPER_IN4 45

// UART Pointers

volatile unsigned char* UCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char* UCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char* UCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int*  UBRR0  = (unsigned  int*) 0x00C4;
volatile unsigned char* UDR0   = (unsigned char*) 0x00C6;

// GPIO Pointers

volatile unsigned char* DDRB   = (unsigned char*) 0x24;
volatile unsigned char* PORTB  = (unsigned char*) 0x25;
volatile unsigned char* DDRH   = (unsigned char*) 0x101;
volatile unsigned char* PORTH  = (unsigned char*) 0x102;

// Functions Declarations
void U0Init(int baud);
unsigned char U0KbHit();
unsigned char U0getChar();
void U0putChar(unsigned char ch);
void printString(const char* str);


// Main Program

LiquidCrystal lcd(PIN_DIS_RS, PIN_DIS_EN, PIN_DIS_D4, PIN_DIS_D5, PIN_DIS_D6, PIN_DIS_D7);

void setup(){
    U0Init(9600);
    lcd.begin(16, 2);
}

void loop(){

}


// UART Functions

void U0Init(int baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / baud - 1);

  // Same as (FCPU / (16 * baud)) - 1;
  *UCSR0A = 0x20;
  *UCSR0B = 0x18;
  *UCSR0C = 0x06;
  *UBRR0  = tbaud;
}

unsigned char U0KbHit(){
    return *UCSR0A & 0x80;
}

unsigned char U0getChar(){
    return *UDR0;
    
}
void U0putChar(unsigned char ch){
  while((*UCSR0A & 0x20) == 0);
  *UDR0 = ch;
}

void printString(const char* str){
    int i = 0;
    while(str[i] != '\0'){
        U0putChar(str[i]);
        i++;
    }
}