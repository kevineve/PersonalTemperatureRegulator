/*Day Hiking Temperature Regulator- By Jason Webster, Brittany Cohen and Kevin Eve
 * Button Interrupt code adapted from https://www.arduino.cc/en/Reference/attachInterrupt
*/ 


#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define IN1_PIN 10
#define POT_PIN 2
#define PIEZO_PIN 8
#define THERMISTER_PIN 0
#define SKIN_TEMP_PIN 1 
#define TARGET_SKIN_TEMP 34
//###### PD Control Constants ############
#define P_GAIN 30
#define D_GAIN 10
#define OFFSET_INCREMENT .01

//######### Sensor Measurement Constants ###########
#define NUMSAMPLES 15
// the value of the 'other' resistor
#define SERIESRESISTOR 10000 

int samples[NUMSAMPLES];

int skin_temp;
int last_skin_temp;
int water_temp;
int pot_value;
int pump_output;
int error;
int derivative;
int offset = 50;

//####### INTERRUPT #########
const byte ledPin = 6;
const byte interruptPin = 3;
volatile byte LCD_Backlight_state = HIGH;

//#### TIMING ########
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

//ADJUST: loop timing period 
const long interval = 200;

//print timing 
unsigned long previousPrintMillis = 0;
const long printInterval = 2000;

//LCD Instantiation
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7, 13,POSITIVE);

void setup() {
  //LCD Initialization 
  lcd.begin (16,2);
  lcd.home ();
  pinMode(ledPin, OUTPUT);

  //Set up Piezo Buzzer
  pinMode(PIEZO_PIN, OUTPUT);
  digitalWrite(PIEZO_PIN,HIGH);

  //Set up Interupt 
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), toggleLCD, CHANGE);
}

void loop() {
  currentMillis = millis();
  if(currentMillis - previousMillis > interval){
    
//    DEMONSTRATION CODE
//    //potentiometer for debugging 
//    pot_value = analogRead(POT_PIN);
//    if(pot_value<20){
//      waterTempWarning();
//    }

    //Read Sensor Values
    water_temp = getWaterTemp();
    skin_temp= getSkinTemp();

    //Set Pump Output 
    pump_output = PdPumpControl(skin_temp);
    analogWrite(IN1_PIN,pump_output);

     //Toggle LCD Backlight debe
    digitalWrite(ledPin, LCD_Backlight_state);

    //If Water Temp is over 20 deg C, Call Warning method
    if(water_temp>20){
        waterTempWarning();
    }
    previousMillis = currentMillis; 
  }
  //print out info at a slower rate than the regular update loop 
  if(currentMillis - previousPrintMillis > printInterval){
    //Print Information to LCD
    printInformation();
    
    previousPrintMillis = currentMillis;
  }
}

//Get pump output value
int PdPumpControl(int skinTemp){
  error = skinTemp - TARGET_SKIN_TEMP; 
  derivative = skinTemp-last_skin_temp;
  
  //if the Skin temp is too high and increasing, increment the offset 
  if(error > 0 && derivative > 0){
    offset = offset + OFFSET_INCREMENT;
  }
  //if the Skin temp is too low and decreasing, decrease the offset 
  if(error < 0 && derivative < 0){
    offset = offset - OFFSET_INCREMENT;
  }
  
  last_skin_temp = skinTemp;
  return (int)(P_GAIN*error + offset);
}

//Method adapted from https://learn.adafruit.com/thermistor/using-a-thermistor
int getWaterTemp(){
  uint8_t i;
  float average;
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTER_PIN);
   delay(10);
  } 
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
 
  float steinhart;
  steinhart = average / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3950;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                

  return steinhart-2;
}

//Method adapted from https://learn.adafruit.com/thermistor/using-a-thermistor
int getSkinTemp(){
  uint8_t i;
  float average;
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(SKIN_TEMP_PIN);
   delay(10);
  } 
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  
  return (average*0.00488-.5)*100 + 2;
}

void toggleLCD(){
  LCD_Backlight_state = !LCD_Backlight_state;
}

void waterTempWarning(){
  //Turn on LCD Backlgiht
  digitalWrite(ledPin, HIGH);
  //Turn on Piezo Buzzer
  digitalWrite(PIEZO_PIN,LOW);
  //Write to LCD
  lcd.clear();
  lcd.print("WARNING: COOLING");
  lcd.setCursor(0, 1);
  lcd.print("SYSTEM DEPLETED");
  delay(1000);
  //Turn off buzzer and LCD backlight
  digitalWrite(PIEZO_PIN,HIGH);
  digitalWrite(ledPin, LOW);
}

void printInformation(){
  lcd.clear();
  lcd.print("Skin Temp: " + String(skin_temp));
  lcd.setCursor(0, 1);
  lcd.print("Water Temp: " + String(water_temp));
//  delay(1600);
//  lcd.clear();
//  lcd.print("Pump Output: " + String(pump_output));
//  lcd.setCursor(0,1);
//  //currentMillis = millis();
//  lcd.print("Uptime: " + String(millis()));
//  delay(1600);
}


