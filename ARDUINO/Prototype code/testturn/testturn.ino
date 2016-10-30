#include <Car.h>
Car car(10, 11, 2, 3, 6, 7, 13, 255, 8, 4);

float M_PI = 3.14159265359;

void setup() {
  // put your setup code here, to run once:
attachInterrupt(car.interruptPin[0], onPulse, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
car.gyroturn(M_PI/2);
car.gyroturn(M_PI/2);
car.gyroturn(-M_PI/2);
car.gyroturn(-M_PI/2);
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
