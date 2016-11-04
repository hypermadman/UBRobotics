#include <Car.h>

#include <LineFollower.h>

int calibratorPins[2];

LineFollower leftSense(48,2);
LineFollower rightSense(34,3);

Car car(22,24,2,3,4,5,6);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  attachInterrupt(0, onPulse, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Loop");
  Serial.println(leftSense.getValue());
  Serial.println(rightSense.getValue());
  
  if (leftSense.getValue() > calibratorPins[0]){ //Turn left
    car.stop();
    delay(100);
    car.turn(50);
    delay(100);
    car.stop();
  }
  else if (rightSense.getValue() > calibratorPins[1]){ //Turn Right
    car.stop();
    delay(100);
    car.turn(-50);
    delay(100);
    car.stop();
  }
   else {
     car.move(50);
   }
   
}

void onPulse() {
  
  if ((car.state[0] == false) && (digitalRead(car.interruptPin[0]) == HIGH)){
    car.counter[0] ++;
    car.state[0] = true;
  }
  else if ((car.state[0] == true) && (digitalRead(car.interruptPin[0]) == LOW)){
    car.counter[0] ++;
    car.state[0] = false;
  }
}
