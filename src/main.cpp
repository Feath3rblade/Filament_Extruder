/*
 * filamentExtruderController
 * Author : Nathan Muszynski
 * Manages temperature and motor speed for filament extruder
 */

//library imports
#include <Arduino.h>
#include <Adafruit_MAX31855.h>
#include <L298N.h>
#include <SPI.h>
#include <Wire.h>


//heater relay control pin and status LED
#define heaterPin 32
#define ledPin 13

//digit segment initialization
//ABCDEFG,dp
const int numeral[10]= {
  B11111100, //0
  B01100000, //1
  B11011010, //2
  B11110010, //3
  B01100110, //4
  B10110110, //5
  B00111110, //6
  B11100000, //7
  B11111110, //8
  B11100110, //9
};

//pins for decimal point and each segment
//dp, G, F, E, D, C, B, A
/*
Pinout of display

D1  A  F  D2  D3  B
8 . 8 . 8 . 8 .
E  D  DP  C  G  D4

*/
const int segmentPins[]= {9,7,3,11,10,8,0,4};
const int numberofDigits=4;
const int digitPins[numberofDigits] = {5,2,1,6}; //digits 1, 2, 3, 4

//temperature control variables
int temperatureSetPoint;
int temperatureSwing = 2; //allowable swing in temperature

//thermocouple reading pins
const int thermocoupleDO = 37;
const int thermocoupleCS = 38;
const int thermocoupleCLK = 39;
Adafruit_MAX31855 thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleDO); //create thermocouple instance

//motor controller pins
const unsigned int motorIN1 = 30;
const unsigned int motorIN2 = 31;
const unsigned int motorEnable = 29;
L298N motor1(motorEnable, motorIN1, motorIN2);

//misc variable declarations
int changeTime = 0;
int tempTemperature;
int motorSpeed = 255;


 
int readPot(){  //read temperature pot value and return as equiv. temperature
  int encoderVal = analogRead(9);
  int equivTemp = (encoderVal/20)*5; //count by 5 from 0 - 255
  
  return equivTemp;
}


void showDigit (int number, int digit){ //displays given number on a 7-segment display at the given digit position
  digitalWrite(digitPins[digit], HIGH);
  for (int segment= 1; segment < 8; segment++){
    boolean isBitSet= bitRead(numeral[number], segment);
    isBitSet= ! isBitSet;
    digitalWrite(segmentPins[segment], isBitSet);
  }
  delay(5);
  digitalWrite(digitPins[digit], LOW);
}


void showNumber (int number){ //takes a number and iterates through it to draw digits on screen
  if (number == 0)
    showDigit (0, numberofDigits-1); //display 0 in the rightmost digit
  else{
    for (int digit= numberofDigits-1; digit >=0; digit--){
      if (number > 0){
        showDigit(number % 10, digit);
        number= number/10;
      }
    }
  }
}


int checkTemp(){ //read temperature from thermocouple and return as integer
  int temperature = thermocouple.readCelsius();
  return temperature;
}


void setTemp(int newTemp){ //set temperature set point to user specified value
  temperatureSetPoint = newTemp;
}


void maintainTemp(int setPoint){
  if (setPoint - checkTemp() >= temperatureSwing){ // if setpoint is higher than actual temperature by over n degrees run heater
    digitalWrite(heaterPin, HIGH);
    digitalWrite(ledPin, HIGH);
  }
  else if (setPoint - checkTemp() >= -temperatureSwing){ //if setpoint is lower than actual temperature by over n degrees, stop heater
    digitalWrite(heaterPin, LOW);
    digitalWrite(ledPin, LOW);
  }
}


void runMotor(int setPoint){
  if (setPoint - checkTemp() <= temperatureSwing){ //if temperature is high enough, run motor
    motor1.forward();
  }
  else{
    motor1.stop();
  }
}

void setup(){
  //set pinmode for all pins
  pinMode(heaterPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  for (int i=0; i < 8; i++){
    pinMode(segmentPins[i], OUTPUT); //set segment and DP pins to output
  }
  
  //sets the digit pins as outputs
  for (int i=0; i < numberofDigits; i++){
    pinMode(digitPins[i], OUTPUT);
  }
  motor1.setSpeed(motorSpeed);
  temperatureSetPoint = readPot();
}




void loop(){
  //check temperature, adjust heater if needed
  maintainTemp(temperatureSetPoint);
  runMotor(temperatureSetPoint);
  
  //if value from potentiometer not equal to setpoint, run temperature change routine
  if (readPot() != temperatureSetPoint){
    showNumber(readPot());

    if (changeTime == 0){ //store current time and temperature value, will wait certain amount of time to check if temperature setpoint changes further
      changeTime = millis(); 
      tempTemperature = readPot();
    }
    
    else if (millis() - changeTime >= 2000){ //n ms after change, check if value is same
      if (tempTemperature == readPot()){
        setTemp(tempTemperature);
        changeTime = 0;
      }
      
      else{ //if changed, reset changeTime and tempTemperature to run new check
        changeTime=millis();
        tempTemperature = readPot();
      }
    }
  }
  else{
    showNumber(checkTemp()); //show current temperature if no change in temperature setting
  }
}
