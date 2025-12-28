//Margaux Cruzan
//CPE 301 final

#include <DHT.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include <Wire.h>

#define RDA 0x80
#define TBE 0x20

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;
// GPIO Pointers for LEDs
volatile unsigned char *portB = (unsigned char *)0x25;
volatile unsigned char *portDDRB = (unsigned char *)0x24;
// GPIO Pointers for start button
volatile unsigned char *portE = (unsigned char *)0x2E;
volatile unsigned char *portDDRE = (unsigned char *)0x2D;
// GPIO Pointers for stop button, fan
volatile unsigned char *portA = (unsigned char *)0x22;
volatile unsigned char *portDDRA = (unsigned char *)0x21;
volatile unsigned char *pinA = (unsigned char *)0x20;
// Timer Pointers
volatile unsigned char *myTCCR1A = (unsigned char *)0x80;
volatile unsigned char *myTCCR1B = (unsigned char *)0x81;
volatile unsigned char *myTCCR1C = (unsigned char *)0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *)0x6F;
volatile unsigned char *myTIFR1 = (unsigned char *)0x36;
volatile unsigned int *myTCNT1 = (unsigned int *)0x84;

#define STATE_DISABLED 0
#define STATE_IDLE 1
#define STATE_RUNNING 2
#define STATE_ERROR 3

//create dht object
#define DHTPIN 10
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

unsigned char currentState = STATE_DISABLED;

float tempReading = 0.0;
float humReading = 0.0;

//delay variables for reading the temp/humidity
unsigned long previousMillis = 0;
const long interval = 60000;
//dealy variables for writing results to LCD
unsigned long previousMillis_1 = 0;
const long interval_1 = 60000;

const float tempThresh = 20.0;
const unsigned int waterThresh = 100; 

const int rs = 30, 
          en = 31,
          d4 = 32,
          d5 = 33,
          d6 = 34,
          d7 = 35;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

Stepper stepper(2048, 36, 38, 37, 39);

int ventPosSteps = 0;
int ventTargetSteps = 0;
const int ventStep = 8;
const int maxDiff = 4;

RTC_DS1307 rtc;

unsigned char prevState;
int prevVentPosSteps;

void startButton(){
  if(currentState == STATE_DISABLED){
    currentState = STATE_IDLE;
  }
}

void adc_init(){
 // set bit 7 to 1 to enable the ADC 
  *my_ADCSRA |= 0b10000000;

 // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11011111;

 // clear bit 3 to 0 to disable the ADC interrupt 
  *my_ADCSRA &= 0b11110111;

 // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= 0b11111000;

  // setup the B register
// clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111;

 // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0b11111000;

  // setup the MUX Register
 // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= 0b01111111;

// set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0b01000000;

  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;

 // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0b11100000;

}
unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;

  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= 0b11110111;
 
  // set the channel selection bits for channel 0
  *my_ADMUX |= (adc_channel_num & 0b00011111);

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;

  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  
  unsigned int val = *my_ADC_DATA;
  return val;
}
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
void print2num(unsigned int v){
  U0putchar('0' + (v / 10));
  U0putchar('0' + (v % 10));
}

void printYear(unsigned int v) {
  char digits[4];
  int index = 0;
  // break the number into digits (backwards)
  while(v > 0){
    digits[index] =(v % 10) + '0';
    index++;
    v = v / 10;
  }
  // print digits in reverse order
  index = index - 1;
  while(index >= 0){
    U0putchar(digits[index]);
    index--;
  }
}

void printDateTime(const DateTime& now){
  printYear(now.year()); 
  U0putchar('-'); 
  print2num(now.month()); 
  U0putchar('-'); 
  print2num(now.day());
  U0putchar(' ');
  print2num(now.hour()); 
  U0putchar(':'); 
  print2num(now.minute()); 
  U0putchar(':'); 
  print2num(now.second());
}

void setup() {

  *portDDRB |= 0b00001111; //pins 50-53 are led outputs
  *portB &= ~0b00001111; //sets all leds low at boot

  currentState = STATE_DISABLED; //sets state to disabled at boot
  //*portB |= 0b00001000; //turns on yellow led 

  //start button
  *portDDRE &= ~(1 << 4); //makes pin 23 start button input
  *portE |= (1 << 4); //makes it so button is high when not pressed

  attachInterrupt(digitalPinToInterrupt(2), startButton, FALLING);

  //stop button
  *portDDRA &= ~(1 << 0); //makes pin 22 input for stop button
  *portA |= (1 << 0); //makes it so button is high when not pressed

  //reset button
  *portDDRA &= ~(1 << 2); //makes pin 24 reset button input
  *portA |= (1 << 2); //makes it so button is high when not pressed

  *portDDRA |= (1 << 1);//sets pin 23 as output for fan
  *portA &= ~(1 << 1);//makes sure fan is off at boot

  dht.begin();
  adc_init();

  lcd.begin(16, 2);
  lcd.clear();

  stepper.setSpeed(10);

  U0init(9600);

  Wire.begin();
  rtc.begin();

  prevState = currentState;
  prevVentPosSteps = ventPosSteps;
}

void loop(){
  //if stop button is pressed, disable
  if(!(*pinA & (1 << 0))){
    currentState = STATE_DISABLED;
  }
  //check water levels if not disabled
  unsigned int waterLev;
  if(currentState != STATE_DISABLED){
    waterLev = adc_read(0);
    //if water is below threshold and its not already in error, enter error
    if(waterLev < waterThresh && currentState != STATE_ERROR){
      currentState = STATE_ERROR;
    }
  //if it is in error and the water is back above the threshold and the reset button was pressed, go back to idle
  if(currentState == STATE_ERROR && waterLev >= waterThresh && !(*pinA & (1 << 2))){
      currentState = STATE_IDLE;
    }
  }
  //if system is running or idle, read temp/humitidy
  if(currentState != STATE_DISABLED && currentState != STATE_ERROR){
    if(millis() - previousMillis >= interval){//delay
      previousMillis = millis();
    
      float t = dht.readTemperature();
      float h = dht.readHumidity();

      if(t >= 0.0 && t <= 50.0){
        tempReading = t;
      }
      if(h >= 0.0 && h <= 100.0){
        humReading = h;
      }
      if(currentState == STATE_IDLE && tempReading >= tempThresh){//if currently idle and temp is too high, switch to running
        currentState = STATE_RUNNING;
      }else if(currentState == STATE_RUNNING && tempReading < tempThresh){//if currently running and temp is too low, switch to idle
        currentState = STATE_IDLE;
      }
    } 
  }
  //sets led state
  if(currentState == STATE_DISABLED){
    *portB &= ~0b00001111;//turns off all leds
    *portB |= 0b00001000;//turns yellow on
  }else if(currentState == STATE_IDLE){
    *portB &= ~0b00001111;//turns off all leds
    *portB |= 0b00000100;//turns green on
  }else if(currentState == STATE_ERROR){
    *portB &= ~0b00001111;//turns off all leds
    *portB |= 0b00000001;//turns red on
  }else if(currentState == STATE_RUNNING){
    *portB &= ~0b00001111;//turns off all leds
    *portB |= 0b00000010;//turns on blues
  }
  //sets fan state
  if(currentState == STATE_RUNNING){
    *portA |= (1 << 1);
  }else{
    *portA &= ~(1 << 1);
  }
  //display on LCD
  //if(currentState != STATE_DISABLED){
    if(millis() - previousMillis_1 >= interval_1){
      previousMillis_1 = millis();

      lcd.clear();

      if(currentState == STATE_ERROR){
        lcd.setCursor(0, 0);
        lcd.print("ERROR: WATER LOW");
      }else if(currentState == STATE_DISABLED){
        lcd.setCursor(0, 0);
        lcd.clear();
        lcd.print("DISABLED");
      }else{
        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(tempReading, 1);
        lcd.print("C H:");
        lcd.print(humReading, 0);
        lcd.print("%");
      }
    }
  //}
  //moves vent
  if(currentState != STATE_DISABLED){
  unsigned int pot = adc_read(1);
  int diff = ventTargetSteps - ventPosSteps;

  if(diff > maxDiff){
    int stepNow;
    if(diff > ventStep){
      stepNow = ventStep;
    }else{
      stepNow = diff;
    }
    stepper.step(stepNow);
    ventPosSteps += stepNow;
  }
  else if(diff < -maxDiff){
    int stepNow;
    if(-diff > ventStep){
      stepNow = ventStep;
    }else{
      stepNow = -diff;
    }
    stepper.step(-stepNow);
    ventPosSteps -= stepNow;
    }
  }

  //log state transitions
  if(currentState != prevState){
    DateTime now = rtc.now();
    printDateTime(now);
    U0putchar(' ');
    char txt[] = " STATE: ";
    for(int i=0; txt[i]; i++){
      U0putchar(txt[i]);
    }
    printInt(prevState);
    U0putchar('-'); 
    U0putchar('>');
    printInt(currentState);
    U0putchar('\n');

    prevState = currentState;
  }

  //l vent position changes
  if(ventPosSteps != prevVentPosSteps){
    DateTime now = rtc.now();
    printDateTime(now);
    U0putchar(' ');
    char txt[] = "VENT POSITION CHANGED\n";
    for(int i = 0; txt[i]; i++){
      U0putchar(txt[i]);
    }

    prevVentPosSteps = ventPosSteps;
  }
}