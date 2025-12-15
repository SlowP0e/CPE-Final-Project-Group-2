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
#include <DHT.h>

// Pin Definitions
#define PIN_LED_BLUE      "PH4"   // PH4 D7  OUT
#define PIN_LED_GREEN     "PH5"   // PH5 D8  OUT
#define PIN_LED_YELLOW    "PH6"   // PH6 D9  OUT
#define PIN_LED_RED       "PB4"   // PB4 D10 OUT

#define PIN_BTN_START     "PE4"   // PE4 D2  IN
#define PIN_BTN_START_ISR 2       // D2, to be used with interrupt
#define PIN_BTN_STOP      "PE5"   // PE5 D3  IN
#define PIN_BTN_RESET     "PG5"   // PG5 D4  IN

#define PIN_FAN           "PB6"   // PB6 D12
#define PIN_DHT11         22      // PA0 D22, to be used with DHT library
#define PIN_WATER_SENSOR  0       // Analog 0, to be used with adcRead()
#define PIN_POTENTIOMETER 1       // Analog 1, to be used with adcRead()

// Pin definitions for clock
#define PIN_CLK_SDA 20      //
#define PIN_CLK_SCL 21      //

// Pin definitions for LCD
#define PIN_DIS_RS 30       // D30
#define PIN_DIS_EN 31       // D31
#define PIN_DIS_D4 32       // D32
#define PIN_DIS_D5 33       // D33
#define PIN_DIS_D6 34       // D34
#define PIN_DIS_D7 35       // D35
#define LCD_UPDATE_INTERVAL 3000UL  // 1 minute

// Pin definitions for stepper motor library
#define STEPPER_STEPS 1024
#define PIN_STEPPER_IN1 42  // D42
#define PIN_STEPPER_IN2 43  // D43
#define PIN_STEPPER_IN3 44  // D44
#define PIN_STEPPER_IN4 45  // D45

// ---------- IDLE thresholds (tune these) ----------
#define WATER_LOW_THRESHOLD   200   // ADC units (0-1023). Pick based on your sensor test.
#define TEMP_HIGH_THRESHOLD_C 20.0  // temp threshold for RUNNING

#define DHTTYPE DHT11

// UART Pointers
volatile unsigned char* _UCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char* _UCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char* _UCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int*  _UBRR0  = (unsigned  int*) 0x00C4;
volatile unsigned char* _UDR0   = (unsigned char*) 0x00C6;

// ADC Pointers
volatile unsigned char* _ADMUX    = (unsigned char*) 0x7C;
volatile unsigned char* _ADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char* _ADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int*  _ADC_DATA = (unsigned int*) 0x78;

enum SystemState {
    STATE_DISABLED,
    STATE_IDLE,
    STATE_ERROR,
    STATE_RUNNING
};

// DHT object
DHT dht(PIN_DHT11, DHTTYPE);

// Prototypes
void handleIdleState();
void handleErrorState();
void handleRunningState();

bool isStopPressed();
bool isResetPressed();
bool isWaterLow();
float readTempC();
float readHumidity();
void uartLog(const char* msg);
void ISR_startButton();

// Functions Declarations
void U0Init(int baud);
unsigned char U0KbHit();
unsigned char U0getChar();
void U0putChar(unsigned char ch);

// Main Program

LiquidCrystal lcd(PIN_DIS_RS, PIN_DIS_EN, PIN_DIS_D4, PIN_DIS_D5, PIN_DIS_D6, PIN_DIS_D7);
unsigned long lastSensorPoll = 0;
Stepper stepper(STEPPER_STEPS, PIN_STEPPER_IN1, PIN_STEPPER_IN3, PIN_STEPPER_IN2, PIN_STEPPER_IN4);
volatile SystemState currentState = STATE_DISABLED;
volatile bool startPressed = false;
int stepperPrev = 0;

void setup(){
    U0Init(9600);
    adcInit();
    lcd.begin(16, 2);
    dht.begin();

    DDRH |= (1 << 4); // BLUE
    DDRH |= (1 << 5); // GREEN
    DDRH |= (1 << 6); // YELLOW
    DDRB |= (1 << 4); // RED
    DDRB |= (1 << 6); // FAN

    DDRE &= ~(1 << 4);
    PORTE |= (1 << 4); // START button
    
    attachInterrupt(digitalPinToInterrupt(2), ISR_startButton, FALLING);

    DDRE &= ~(1 << 5);
    PORTE |= (1 << 5); // STOP button

    DDRG &= ~(1 << 5);
    PORTG |= (1 << 5); // RESET button

    PORTH &= ~(1 << 4); // BLUE OFF
    PORTH &= ~(1 << 5); // GREEN OFF
    PORTH &= ~(1 << 6); // YELLOW OFF
    PORTB &= ~(1 << 4); // RED OFF
    PORTB &= ~(1 << 6); // FAN OFF

    stepper.setSpeed(26);
    stepperPrev = adcRead(PIN_POTENTIOMETER);

}

void loop(){
    switch (currentState) {
        case STATE_DISABLED: handleDisabledState(); break;
        case STATE_IDLE:     handleIdleState();     break;
        case STATE_ERROR:    handleErrorState();    break;
        case STATE_RUNNING:  handleRunningState();  break;
    }

    int potValue = adcRead(PIN_POTENTIOMETER);
    int stepVal = potValue - stepperPrev;
    if(stepVal != 0) stepper.step(stepVal);
    stepperPrev = potValue;
}

void ISR_startButton() {
    startPressed = true; // ISR start button
}

// State Handlers

void handleDisabledState() {
    static bool firstEntry = true;

    if (firstEntry) {
        // YELLOW ON, all others OFF
        PORTH |=  (1 << 6);  // YELLOW ON
        PORTH &= ~((1 << 4) | (1 << 5)); // BLUE & GREEN OFF
        PORTB &= ~(1 << 4);  // RED OFF 

        // Fan OFF
        PORTB &= ~(1 << 6);

        // LCD message
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SYSTEM");
        lcd.setCursor(0, 1);
        lcd.print("DISABLED");

        uartLog("[STATE] ENTER DISABLED");

        firstEntry = false;
    }

    // Only action in disabled state: start button
    if (startPressed) {
      lastSensorPoll = 0;        // LCD update

      firstEntry = true;        // allow re-init next time
      currentState = STATE_IDLE;

      startPressed = false;
    }
}

bool isStopPressed() {
  bool isPressed = (PINE & (1 << 5)) == 0;

  if(isPressed){
    do {
      isPressed = (PINE & (1 << 5)) == 0;
      delay(50);
    } while(isPressed);

    return true;
  }
  return false;
}

bool isResetPressed() {
  bool isPressed = (PING & (1 << 5)) == 0;

  if(isPressed){
    do {
      isPressed = (PING & (1 << 5)) == 0;
      delay(50);
    } while(isPressed);

    return true;
  }
  return false;
}

bool isWaterLow() {
  unsigned int w = adcRead(PIN_WATER_SENSOR);
  return (w < WATER_LOW_THRESHOLD);
}

// Replaced stub logic with real DHT11 reads
float readTempC() {
  float t = dht.readTemperature();
  if (isnan(t)) {
    uartLog("[WARN] DHT temp read failed");
    return -999.0f;
  }
  return t;
}

float readHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    uartLog("[WARN] DHT humidity read failed");
    return -1.0f;
  }
  return h;
}

void uartLog(const char* msg) {
  int i = 0;
  while(msg[i] != '\0'){
      U0putChar(msg[i]);
      i++;
  }
  U0putChar('\n');
}

void handleIdleState() {
  static bool firstEntry = true;
  unsigned long now = millis();

  if (firstEntry) {
    // GREEN ON, all others OFF
    PORTH |=  (1 << 5);
    PORTH &= ~((1 << 4) | (1 << 6));
    PORTB &= ~(1 << 4);

    // Fan OFF
    PORTB &= ~(1 << 6);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IDLE");

    uartLog("[STATE] ENTER IDLE");
    lastSensorPoll = 0;
    firstEntry = false;
  }

  // STOP if detects stop button
  if (isStopPressed()) {
    uartLog("[EVENT] STOP -> DISABLED");
    PORTB &= ~(1 << 6);   // Fan OFF
    firstEntry = true;
    currentState = STATE_DISABLED;
    return;
  }

  // Water level monitored continuously
  if (isWaterLow()) {
    uartLog("[EVENT] WATER LOW -> ERROR");
    PORTB &= ~(1 << 6); // Fan OFF
    firstEntry = true;
    currentState = STATE_ERROR;
    return;
  }

  // LCD Display
  if (now - lastSensorPoll >= LCD_UPDATE_INTERVAL) {
    float t_now = readTempC();
    if (t_now > TEMP_HIGH_THRESHOLD_C) {
      uartLog("[EVENT] TEMP HIGH -> RUNNING");
      firstEntry = true;
      currentState = STATE_RUNNING;
      return;
    }
    float h = readHumidity();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T: ");
    lcd.print(t_now, 1);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("H: ");
    lcd.print(h, 0);
    lcd.print("%");

    lastSensorPoll = now;
  }
}


void handleRunningState() {
  static bool firstEntry = true;
  unsigned long now = millis();

  if (firstEntry) {
    // BLUE ON, all others OFF
    PORTH |=  (1 << 4);
    PORTH &= ~((1 << 5) | (1 << 6));
    PORTB &= ~(1 << 4);

    // Fan ON
    PORTB |= (1 << 6);

    // Force LCD to update immediately on entry
    lastSensorPoll = 0;

    firstEntry = false;
  }

  // Water level has highest priority
  if (isWaterLow()) {
    PORTB &= ~(1 << 6); // Fan OFF
    firstEntry = true;
    currentState = STATE_ERROR;
    return;
  }

  if(isStopPressed()) {
    uartLog("[EVENT] STOP -> DISABLED");
    PORTB &= ~(1 << 6);   // Fan OFF
    currentState = STATE_DISABLED;
    return;
  }

  // LCD shows Temperature + Water reading (timed)
  if (now - lastSensorPoll >= LCD_UPDATE_INTERVAL) {
    float t_now = readTempC();
    if (t_now <= TEMP_HIGH_THRESHOLD_C) {
      PORTB &= ~(1 << 6); // Fan OFF
      firstEntry = true;
      currentState = STATE_IDLE;
      return;
    }
    float h = readHumidity();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(t_now, 1);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("H: ");
    lcd.print(h, 0);
    lcd.print("%");

    lastSensorPoll = now;
  }
}


void handleErrorState() {
  static bool firstEntry = true;

  if (firstEntry) {
    PORTB |=  (1 << 4); // RED ON
    PORTH &= ~((1 << 4) | (1 << 5)); // BLUE, GREEN OFF
    PORTH &= ~(1 << 6); // YELLOW OFF

    // Fan OFF
    PORTB &= ~(1 << 6);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR");
    lcd.setCursor(0, 1);
    lcd.print("Low Water!");

    firstEntry = false;
  }

  if (isResetPressed() && !isWaterLow()) {
    firstEntry = true;
    currentState = STATE_IDLE;
  }
}

// ADC Functions

void adcInit(){
  *_ADCSRA |= 0b10000000;
  *_ADCSRA &= 0b11011111;
  *_ADCSRA &= 0b11110111;
  *_ADCSRA &= 0b11111000;
  *_ADCSRA |= 0b00000111;

  *_ADCSRB &= 0b11110111;
  *_ADCSRB &= 0b11111000;

  *_ADMUX &= 0b01111111;
  *_ADMUX |= 0b01000000;
  *_ADMUX &= 0b11011111;
  *_ADMUX &= 0b11100000;
}

unsigned int adcRead(unsigned char channel){
  *_ADMUX &= 0b11100000;
  *_ADCSRB &= 0b11110111;
  *_ADMUX |= (channel & 0x1F);
  *_ADCSRA |= 0b01000000;

  while((*_ADCSRA & 0x40) != 0);
  return (*_ADC_DATA & 0x03FF);
}

// UART Functions

void U0Init(int baud){
    unsigned long FCPU = 16000000;
    unsigned int tbaud = (FCPU / 16 / baud - 1);

    *_UCSR0A = 0x20;
    *_UCSR0B = 0x18;
    *_UCSR0C = 0x06;
    *_UBRR0  = tbaud;
}

unsigned char U0KbHit(){
    return *_UCSR0A & 0x80;
}

unsigned char U0getChar(){
    return *_UDR0;
}

void U0putChar(unsigned char ch){
    while((*_UCSR0A & 0x20) == 0);
    *_UDR0 = ch;
}
