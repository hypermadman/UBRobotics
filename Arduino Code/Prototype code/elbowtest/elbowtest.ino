#include <Hinge.h>

Hinge elbow(52,53,A0);
Hinge shoulder(50,51,A1);

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
elbow.turnTo(900);
delay(500);
shoulder.turnTo(500);
}

void loop() {
  // put your main code here, to run repeatedly:
}
