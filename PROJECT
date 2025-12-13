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

#define PIN_LED_BLUE   7    // PH4 D7. OUT
#define PIN_LED_GREEN  8    // PH5 D8. OUT
#define PIN_LED_YELLOW 9    // PH6 D9. OUT
#define PIN_LED_RED    10   // PB4 D10 OUT

#define PIN_BTN_START  2    // PE4 D2  IN
#define PIN_BTN_STOP   3    // PE5 D3  IN
#define PIN_BTN_CLEAR  4    // PG5 D4  IN

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

// ---------- IDLE thresholds (tune these) ----------
#define WATER_LOW_THRESHOLD   200   // ADC units (0-1023). Pick based on your sensor test.
#define TEMP_HIGH_THRESHOLD_C 20.0  // temp threshold for RUNNING

unsigned long lastLCDUpdate = 0;
extern LiquidCrystal lcd;

// Prototypes
void initIdleStateHardware();
void handleIdleState();
void handleErrorState();   // ERROR state handler
void handleRunningState(); // RUNNING state handler

bool isStopPressed();
bool isClearPressed();     // CLEAR button (reset)
bool isWaterLow();
float readTempC_stub();
float readHumidity_stub();
void printString(const char* str);   // <-- MUST be above uartLog()
void uartLog(const char* msg);

enum SystemState {
    STATE_DISABLED,
    STATE_IDLE,
    STATE_ERROR,
    STATE_RUNNING
};

volatile SystemState currentState = STATE_DISABLED;
volatile bool startPressed = false;

void ISR_startButton() {
    startPressed = true; // ISR start button
}

void initDisabledStateHardware() {

    // Yellow LED = PIN_LED_YELLOW = D9 = PH6
    DDRH |= (1 << 6);    // Sets PH6 as OUTPUT
    PORTH |= (1 << 6);   // LED ON at startup (DISABLED)

    DDRE &= ~(1 << 4);   // PE4 input
    PORTE |= (1 << 4);   // Enable pull-up

    // Attachs interrupt to INT4
    // digitalPinToInterrupt(2) == 4
    attachInterrupt(digitalPinToInterrupt(2), ISR_startButton, FALLING);
}

void handleDisabledState() {

    // Keeps yellow LED ON while disabled
    PORTH |= (1 << 6);

    // Only action in disabled state: start button ISR triggers transition
    if (startPressed) {
        startPressed = false;

        // Turns off yellow LED when leaving disabled state
        PORTH &= ~(1 << 6);

        currentState = STATE_IDLE;
    }
}

void initIdleStateHardware() {
  // LEDs: Blue PH4(D7), Green PH5(D8), Yellow PH6(D9), Red PB4(D10)
  DDRH |= (1 << 4) | (1 << 5) | (1 << 6);  // PH4/PH5/PH6 outputs
  DDRB |= (1 << 4);                        // PB4 output

  // Fan: PB6 (D12) output
  DDRB |= (1 << 6);

  // Stop button: PE5 (D3) input with pull-up
  DDRE &= ~(1 << 5);
  PORTE |= (1 << 5);

  // Clear button: PG5 (D4) input with pull-up
  DDRG &= ~(1 << 5);
  PORTG |= (1 << 5);
}

bool isStopPressed() {
  // Active LOW with pull-up
  return ((PINE & (1 << 5)) == 0);
}

bool isClearPressed() {
  // Active LOW with pull-up
  return ((PING & (1 << 5)) == 0);
}

bool isWaterLow() {
  unsigned int w = adcRead(PIN_WATER_SENSOR); // A0 channel 0
  return (w < WATER_LOW_THRESHOLD);
}

// Stubs for now (so your code compiles). Replace with DHT11 reads later.
float readTempC_stub()    { return 24.0; }
float readHumidity_stub() { return 40.0; }

void uartLog(const char* msg) {
  printString(msg);
  printString("\n");
}

void handleIdleState() {
  static bool firstEntry = true;
  unsigned long now = millis();

  if (firstEntry) {
    initIdleStateHardware();
    uartLog("[STATE] ENTER IDLE");

    // GREEN ON, others OFF
    PORTH |=  (1 << 5);  // Green PH5 ON
    PORTH &= ~(1 << 4);  // Blue  PH4 OFF
    PORTH &= ~(1 << 6);  // Yellow PH6 OFF
    PORTB &= ~(1 << 4);  // Red   PB4 OFF

    // Fan OFF
    PORTB &= ~(1 << 6);  // PB6 LOW

    lastLCDUpdate = now;
    firstEntry = false;
  }

  // STOP if detects stop button
  if (isStopPressed()) {
    uartLog("[EVENT] STOP -> DISABLED");
    // Ensure fan off and LEDs handled by disabled state
    PORTB &= ~(1 << 6);
    firstEntry = true;
    currentState = STATE_DISABLED;
    return;
  }

  // Water level monitored continuously
  if (isWaterLow()) {
    uartLog("[EVENT] WATER LOW -> ERROR");
    PORTB &= ~(1 << 6); // fan off if necessary
    firstEntry = true;
    currentState = STATE_ERROR;
    return;
  }

  // LCD update once per minute 
  if (now - lastLCDUpdate >= 60000UL) {
    float t = readTempC_stub();
    float h = readHumidity_stub();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(t, 1);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("H:");
    lcd.print(h, 0);
    lcd.print("%");

    lastLCDUpdate = now;

    if (t > TEMP_HIGH_THRESHOLD_C) {
      uartLog("[EVENT] TEMP HIGH -> RUNNING");
      firstEntry = true;
      currentState = STATE_RUNNING;
      return;
    }
  }
}

void handleRunningState() {
  static bool firstEntry = true;

  if (firstEntry) {
    uartLog("[STATE] ENTER RUNNING");

    // BLUE ON, all others OFF
    PORTH |=  (1 << 4);                        // Blue PH4 ON
    PORTH &= ~((1 << 5) | (1 << 6));           // Green & Yellow OFF
    PORTB &= ~(1 << 4);                        // Red OFF

    // Fan ON
    PORTB |= (1 << 6);                         // PB6 HIGH

    firstEntry = false;
  }

  // Water level has priority
  if (isWaterLow()) {
    PORTB &= ~(1 << 6); // Fan OFF
    firstEntry = true;
    currentState = STATE_ERROR;
    return;
  }

  // Return to IDLE if temperature drops
  if (readTempC_stub() <= TEMP_HIGH_THRESHOLD_C) {
    PORTB &= ~(1 << 6); // Fan OFF
    firstEntry = true;
    currentState = STATE_IDLE;
    return;
  }
}

void handleErrorState() {
  static bool firstEntry = true;

  if (firstEntry) {
    uartLog("[STATE] ENTER ERROR");

    // RED ON, all others OFF
    PORTH &= ~((1 << 4) | (1 << 5) | (1 << 6)); // Blue, Green, Yellow OFF
    PORTB |=  (1 << 4);                        // Red ON

    // Fan OFF
    PORTB &= ~(1 << 6);

    // Error message on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR");
    lcd.setCursor(0, 1);
    lcd.print("Low Water!");

    firstEntry = false;
  }

  // Reset button returns to IDLE if water is OK
  if (isClearPressed() && !isWaterLow()) {
    uartLog("[EVENT] RESET -> IDLE");
    firstEntry = true;
    currentState = STATE_IDLE;
  }
}

// UART Pointers
volatile unsigned char* pUCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char* pUCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char* pUCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int*  pUBRR0  = (unsigned  int*) 0x00C4;
volatile unsigned char* pUDR0   = (unsigned char*) 0x00C6;

// ADC Pointers
volatile unsigned char* pADMUX    = (unsigned char*) 0x7C;
volatile unsigned char* pADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char* pADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int*  pADC_DATA = (unsigned int*) 0x78;

// GPIO Pointers
volatile unsigned char* pDDRB  = (unsigned char*) 0x24;
volatile unsigned char* pPORTB = (unsigned char*) 0x25;
volatile unsigned char* pDDRH  = (unsigned char*) 0x101;
volatile unsigned char* pPORTH = (unsigned char*) 0x102;

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
    initDisabledStateHardware();
    
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
    switch (currentState) {

        case STATE_DISABLED:
            handleDisabledState();
            return;

        case STATE_IDLE:
            handleIdleState();
            return;

        case STATE_ERROR:
            handleErrorState();
            return;

        case STATE_RUNNING:
            handleRunningState();
            return;
    }

    int potValue = adcRead(PIN_POTENTIOMETER);
    int stepVal = potValue - stepperPrev;
    if(stepVal != 0) stepper.step(stepVal);
    stepperPrev = potValue;
}

// ADC Functions

void adcInit(){
  *pADCSRA |= 0b10000000;
  *pADCSRA &= 0b11011111;
  *pADCSRA &= 0b11110111;
  *pADCSRA &= 0b11111000;
  *pADCSRA |= 0b00000111;
  
  *pADCSRB &= 0b11110111;
  *pADCSRB &= 0b11111000;
  
  *pADMUX &= 0b01111111;
  *pADMUX |= 0b01000000;
  *pADMUX &= 0b11011111;
  *pADMUX &= 0b11100000;
}

unsigned int adcRead(unsigned char channel){
  *pADMUX &= 0b11100000;
  *pADCSRB &= 0b11110111;
  *pADMUX |= (channel & 0x1F);
  *pADCSRA |= 0b01000000;

  while((*pADCSRA & 0x40) != 0);
  
  return (*pADC_DATA & 0x03FF);
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
