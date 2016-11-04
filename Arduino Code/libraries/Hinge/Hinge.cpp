#include "Hinge.h"
#include "Arduino.h"


Hinge::Hinge(int current, int direction, int sensor){
pinMode(current, OUTPUT);
pinMode(direction, OUTPUT);

currentPin = current;
directionPin = direction;
sensorPin = sensor;
}

void Hinge::turnTo(int target) {
  // put your main code here, to run repeatedly:
int current = analogRead(sensorPin);

  while (abs(current - target) > 10){
    current = analogRead(sensorPin);

    if (current > target){ 
      digitalWrite(currentPin, HIGH);
      digitalWrite(directionPin, HIGH);
    }
    else if (current < target){ 
      digitalWrite(currentPin, HIGH);
      digitalWrite(directionPin, LOW);
    }
    Serial.println(current);
    delay(10);
  }
    digitalWrite(currentPin, LOW);
}