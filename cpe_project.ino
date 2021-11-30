#include <LiquidCrystal.h>
#include <DHT.h>
#include "wiring_private.h"
#include "pins_arduino.h"

#include <Wire.h>
#include <DS3231.h>
#include <dht_nonblocking.h>

DS3231 clock;
DateTime dt;

int flag = 1;

#define WATER_LEVEL_THRESHOLD 30 //insert appropriate value here
#define lowThreshold 30
#define highThreshold 50

/* Uncomment according to your sensortype. */
#define DHT_SENSOR_TYPE DHT_TYPE_11
//#define DHT_SENSOR_TYPE DHT_TYPE_21
//#define DHT_SENSOR_TYPE DHT_TYPE_22

static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );


//interrupts
volatile const uint8_t buttonPin = 2;
volatile const uint8_t ledPin = 5;

//define Port A register pointers
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

//define Port B register pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

//define ADC
volatile unsigned char* my_ADMUX = 0x7C;
volatile unsigned char* my_ADCSRB = 0x7B;
volatile unsigned char* my_ADCSRA = 0x7A;
volatile unsigned int* my_ADC_DATA = 0x78;
unsigned char bits_ADMUX[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

//RGB LED
volatile unsigned char* port_L = (unsigned char*) 0x10B;
volatile unsigned char* ddr_L  = (unsigned char*) 0x10A;
volatile unsigned char* pin_L  = (unsigned char*) 0x109;
//pointer to Timer 5
volatile unsigned char* TCCR_5C  = (unsigned char*) 0x120;

int Red = 5; // 44
int Green = 4; // 45
int Blue = 3; // 46

int red_light_pin = 5;
int green_light_pin = 4;
int blue_light_pin = 3;

// Output Compare Pins - Turns on Alternatve Functionality ↓
int COM_5_B1 = 5;
// Wave Form Generation Mode - Turns on PWM in 8 bit mode (mode 1);
int WGM_5_1 = 1;

// Output Compare Register 5 B - This is where we store the Duty Cycle
volatile unsigned char* OCR_5B  = (unsigned char*) 0x12A;

//define states
const int disable = 0;
const int idle = 1;
const int RUNNING = 2;
const int error = 3; 

int adc_id = 0;
int HistoryValue = 0;
char printBuffer[128];
int tempPin = 0;

static void measure_environment( float *temperature, float *humidity );
bool checkWaterLevel();

LiquidCrystal lcd(31, 32, 33, 34, 35, 36);

void disabledState(){
    // sends time for beginning of state
//  sendTime();
    // keeps LCD blank
  lcd.clear();
  lcd.noDisplay();
  
    // turns on yellow LED on pin 8
  *port_b = 0B00000001;

// check for if start button is pressed
}

void setup(){
// Sets Pins as Outputs
  *ddr_L |= 0x01 << Red;
  *ddr_L |= 0x01 << Green;
  *ddr_L |= 0x01 << Blue;

  lcd.begin(16, 2);
  
  //adc_init();

  Serial.begin(9600);

  // Initialize DS3231
  //clock.begin();

  /*
    // Manual (YYYY, MM, DD, HH, II, SS
    // might be able to just comment this line out during testing
  clock.setDate(2021, 12, 06, 05, 30, 00);
  
  // Send sketch compiling time to Arduino
  clock.setDate(__DATE__, __TIME__); */

*ddr_b &= (1 << buttonPin);
*port_b |= (1 << ledPin);

*ddr_b |=  (1 << ledPin);

EICRA |= (1 << ISC01);
EICRA &= (1 << ISC00);

EIMSK |= (1 << INT0);

  //set PB7 to OUTPUT
  *ddr_b |= 0x80;

  // set PB6 (button) to INPUT
  *ddr_b &= (0x01 << 6);
  // Initialize PB6 to low
  *port_b &= ~(0x01 << 6); 
}

void loop() {
  //yellow = 255, 150, 0
  RGB_color(255,150,0);

  if(flag){
    *port_b ^= (1 << ledPin);
     flag = 0;
  }
  // turn on yellow LED for disabled state
  //*port_b = 0B00000010;
  // pin 12
  //*pin_b = 0B00100000;
  float temperature = 0;
  float humidity = 0;
  

  measure_environment(&temperature, &humidity);

  //TODO: find thresholds from serial monitoring

  //runningState
  while(temperature > highThreshold && checkWaterLevel()){
      //runningState()
      //set LED blue
     // *port_b = 0B00000100;
  //    sendTime();
      measure_environment(&temperature, &humidity);
  }

  //idleState
  if(temperature < lowThreshold){
      //set LED green
      //*port_b = 0B00010000;
//      sendTime();
      measure_environment(&temperature, &humidity);
  }

  //errorState
  while(!checkWaterLevel()){
      //set LED red
      //*port_b = 0B00001000;
//      sendTime();
      //turn off motor
      
      measure_environment(&temperature, &humidity);
  }
}

ISR(INT0_vect){
  *port_b ^= (1 << ledPin);
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  aWrite(red_light_pin, red_light_value);
  aWrite(green_light_pin, green_light_value);
  aWrite(blue_light_pin, blue_light_value);
}

void LED_color(char color) {
  // r = Red
  // g = Green
  // b = Blue
  // y = Yellow

write_pL(Red, 1);
write_pL(Green, 0);
write_pL(Blue, 0);

  switch (color) {

    case 'r':
      write_pL(Red, 1);
      break;
    case 'g':
      write_pL(Green, 1);
      break;
    case 'b':
      write_pL(Blue, 1);
      break;
    /*case 'y':      
      write_pL(Red, 1);
      *TCCR_5C |= (1 << COM_5_B1) | (1 << WGM_5_1);
      *OCR_5B = 90;
      break;*/
  }

}

void write_pL(unsigned char pin_num, unsigned char state)
{
  if (state == 0){
    *port_L &= ~(0x01 << pin_num);
  
  }else{
    *port_L |= 0x01 << pin_num;
  
  }
}

/*
void sendTime(){
  dt = clock.getDateTime();

  // For leading zero look to DS3231_dateformat example

  Serial.print("Switch state time: ");
  Serial.print(dt.year);   Serial.print("-");
  Serial.print(dt.month);  Serial.print("-");
  Serial.print(dt.day);    Serial.print(" ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");

  delay(1000);
}
*/
    //int value = readWater(adc_id); // get adc value

    /*if(((HistoryValue>=value) && ((HistoryValue - value) > 10)) || ((HistoryValue<value) && ((value - HistoryValue) > 10)))
    {
      sprintf(printBuffer,"ADC%d level is %d\n",adc_id, value);
      Serial.print(printBuffer);
      HistoryValue = value;
    }
}
*/

void adc_init()
{
  // set up the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 3 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 2-0 to 0 to set prescaler selection to slow reading
  
  // set up the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111; // clear bit 2-0 to 0 to set free running mode
  
  // set up the MUX Register
  *my_ADMUX  &= 0b00000000; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

bool checkWaterLevel(){
  int value = adc_read(adc_id); // get adc value
  int threshold = 0;

  if(((HistoryValue>=value) && ((HistoryValue - value) > 10)) || ((HistoryValue<value) && ((value - HistoryValue) > 10))) {
    HistoryValue = value;
  }

  //TODO: define water level threshold

  (value > threshold) ? true : false;
}

static void measure_environment( float *temperature, float *humidity )
{
  static unsigned int measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
    }
  }

    Serial.print("T: ");
   // Serial.print(temperature, 1);
    Serial.print("C, H: ");
    //Serial.print(humidity, 1);
    Serial.print("%");
}


/*
//copied from ELEGOO lesson 23: thermometer
int checkTemperature(){
  int tempReading = adc_read(tempPin);
  // This is OK
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  float tempC = tempK - 273.15;            // Convert Kelvin to Celcius
  
  // Display Temperature in C
  //lcd.setCursor(0, 0);
  //lcd.print("Temp         C  ");
 // lcd.setCursor(6, 0);
  // Display Temperature in C
  lcd.print(tempC);
  delay(500);
}

int checkHumidity(){

}

void printTempHumidity(){

}
*/

void sendRTCTime(){

}
/*
void analogReference(uint8_t mode)
{
  // can't actually set the register here because the default setting
  // will connect AVCC and the AREF pin, which would cause a short if
  // there's something connected to AREF.
  analog_reference = mode;
}
*/
/*
int adc_read(uint8_t pin)
{
  uint8_t low, high;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
  if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
  if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#elif defined(analogPinToChannel) && (defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__))
  pin = analogPinToChannel(pin);
#else
  if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif
  
#if defined(__AVR_ATmega32U4__)
  pin = analogPinToChannel(pin);
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#elif defined(ADCSRB) && defined(MUX5)
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
  
  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
#if defined(ADMUX)
  ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif

  // without a delay, we seem to read from the wrong channel
  //delay(1);

#if defined(ADCSRA) && defined(ADCL)
  // start the conversion
  sbi(ADCSRA, ADSC);

  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low  = ADCL;
  high = ADCH;
#else
  // we dont have an ADC, return 0
  low  = 0;
  high = 0;
#endif

  // combine the two bytes
  return (high << 8) | low;
}
*/

unsigned int adc_read(unsigned char adc_channel_num)
{
  // reset the channel and gain bits
  *my_ADMUX &= 0b11100000;
  
  // clear the channel selection bits
  *my_ADCSRB &= 0b11110111;
  
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    
    // set MUX bit 
    *my_ADCSRB |= 0b00001000;
  }
  
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  
  // set bit ?? of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;
  
  // wait for the conversion to complete
  while(*my_ADCSRA & 0b000000000);
  
  // return the result in the ADC data register
  return *my_ADC_DATA;
}






//____________________________________________________________________________________________________________________________________________________________________________________________________________
/*
//www.elegoo.com
//2016.12.9

/*
  LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

  The circuit:
 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital pin 8
 * LCD D4 pin to digital pin 9
 * LCD D5 pin to digital pin 10
 * LCD D6 pin to digital pin 11
 * LCD D7 pin to digital pin 12
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(4, 5, 6, 7, 8, 9);

#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 49;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );


void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Hello, World!");
  Serial.begin(9600);
}

static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds.
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}

void loop() {
  float temperature;
  float humidity;

  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. 
  if( measure_environment( &temperature, &humidity ) == true )
  {
    Serial.print( "T = " );
    Serial.print( temperature, 1 );
    Serial.print( " deg. C, H = " );
    Serial.print( humidity, 1 );
    Serial.println( "%" );
    lcd.setCursor(0,0);
    lcd.print("T = ");
    lcd.print(temperature, 1);
    lcd.print(" deg. C");
    lcd.setCursor(0,1);
    lcd.print("H = ");
    
    lcd.print(humidity, 1);
    lcd.print("%");
  }
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  //lcd.print((millis()^10)*100);
}
*/


//__________________________________________________________________________________________
// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void aWrite(uint8_t pin, int val)
{
  // We need to make sure the PWM output is enabled for those pins
  // that support it, as we turn it off when digitally reading or
  // writing with them.  Also, make sure the pin is in output mode
  // for consistenty with Wiring, which doesn't require a pinMode
  // call for the analog output pins.
  pMode(pin, OUTPUT);
  if (val == 0)
  {
    dWrite(pin, LOW);
  }
  else if (val == 255)
  {
    dWrite(pin, HIGH);
  }
  else
  {
    switch(digitalPinToTimer(pin))
    {
      case TIMER0A:
        // connect pwm to pin on timer 0, channel A
        sbi(TCCR0A, COM0A1);
        OCR0A = val; // set pwm duty
        break;

      case TIMER0B:
        // connect pwm to pin on timer 0, channel B
        sbi(TCCR0A, COM0B1);
        OCR0B = val; // set pwm duty
        break;

      case TIMER1A:
        // connect pwm to pin on timer 1, channel A
        sbi(TCCR1A, COM1A1);
        OCR1A = val; // set pwm duty
        break;

      case TIMER1B:
        // connect pwm to pin on timer 1, channel B
        sbi(TCCR1A, COM1B1);
        OCR1B = val; // set pwm duty
        break;

      case TIMER2A:
        // connect pwm to pin on timer 2, channel A
        sbi(TCCR2A, COM2A1);
        OCR2A = val; // set pwm duty
        break;

      case TIMER2B:
        // connect pwm to pin on timer 2, channel B
        sbi(TCCR2A, COM2B1);
        OCR2B = val; // set pwm duty
        break;

      case NOT_ON_TIMER:
      default:
        if (val < 128) {
          dWrite(pin, LOW);
        } else {
          dWrite(pin, HIGH);
        }
    }
  }
}



void dWrite(uint8_t pin, uint8_t val)
{
        uint8_t timer = digitalPinToTimer(pin);
        uint8_t bit = digitalPinToBitMask(pin);
        uint8_t port = digitalPinToPort(pin);
        volatile uint8_t *out;

        if (port == NOT_A_PIN) return;

        // If the pin that support PWM output, we need to turn it off
        // before doing a digital write.
        //if (timer != NOT_ON_TIMER) turnOffPWM(timer);

        out = portOutputRegister(port);

        uint8_t oldSREG = SREG;
        cli();

        if (val == LOW) {
                *out &= ~bit;
        } else {
                *out |= bit;
        }

        SREG = oldSREG;
}

void pMode(uint8_t pin, uint8_t mode)
{
        uint8_t bit = digitalPinToBitMask(pin);
        uint8_t port = digitalPinToPort(pin);
        volatile uint8_t *reg, *out;

        if (port == NOT_A_PIN) return;

        // JWS: can I let the optimizer do this?
        reg = portModeRegister(port);
        out = portOutputRegister(port);

        if (mode == INPUT) {
                uint8_t oldSREG = SREG;
                cli();
                *reg &= ~bit;
                *out &= ~bit;
                SREG = oldSREG;
        } else if (mode == INPUT_PULLUP) {
                uint8_t oldSREG = SREG;
                cli();
                *reg &= ~bit;
                *out |= bit;
                SREG = oldSREG;
        } else {
                uint8_t oldSREG = SREG;
                cli();
                *reg |= bit;
                SREG = oldSREG;
        }
}
