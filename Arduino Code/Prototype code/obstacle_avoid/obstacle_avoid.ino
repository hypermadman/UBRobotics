#include <NewPing.h>

#include <Car.h>

NewPing ping(26, 25, 60);

Car car(22, 24, 2, 3, 4, 5, 6);

void setup() {
  // put your setup code here, to run once:
attachInterrupt(0, onPulse, CHANGE);
car.move(255);

Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((ping.ping_cm() <= 15)&&(ping.ping_cm() != 0)){
    car.stop();
    delay(50);
    car.move(-200);
    delay(1000);
    car.stop();
    delay(50);
    car.turn(250);
  }
  else {
    car.move(255);
  }
  Serial.println(ping.ping_cm());
  delay(50);
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
