#include "LineFollower.h"
#include "Arduino.h"

LineFollower::LineFollower(int LED, int SENSOR){
  LEDpin = LED;
  sensorPin = SENSOR;
  pinMode(LEDpin, OUTPUT);
  
  digitalWrite(LEDpin, HIGH);
}

int LineFollower::getValue(){
  delay(50);
  int reading = analogRead(sensorPin);
  Serial.println(reading);
  delay(50);
  return reading;
}