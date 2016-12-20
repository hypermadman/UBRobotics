//Sketch by Daniel Hingston 

#include <Servo.h>
#include <NewPing.h>

Servo myservo; 

int angle;
int distance;
int pos = 40;
int minRange =1000;
int minRange2 = 1000;
int minIndex;
int minIndex2;
int finalangle;
int finaldistance;
NewPing ping(10,11,1000);

void setup() {
  myservo.attach(2); 
  Serial.begin(9600); 
}

void loop() {
  myservo.write(pos);
  delay(150);
  
  for (pos = 40; pos <= 140; pos += 4) {
    myservo.write(pos);          
    delay(10);
    distance=(ping.ping_cm());
    delay(30);
    if (distance != 0) {
    if (minRange>distance){ 
      minRange = distance;
      minIndex = pos;
    }
    }
   }

  delay(50);
  
  for (pos = 140; pos >= 40; pos -= 4) {
    myservo.write(pos);          
    delay(10);
    distance=(ping.ping_cm());
    delay(30);
    if (distance != 0) {
    if (minRange2>distance){ 
      minRange2 = distance;
      minIndex2 = pos;
    }
    }
   }
   
   finaldistance = (minRange+minRange2)/2;
   finalangle = (minIndex+minIndex2)/2;
 Serial.println(finaldistance);
 Serial.println(finalangle);
 Serial.println();
}

