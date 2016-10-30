#include <NewPing.h>

int shortDist = 6;
int longDist  = 12;

NewPing forward(4, 3, 30);
NewPing rear(5, 2, 30);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int distF = forward.ping_cm();
  delay(50);
  int distR = rear.ping_cm();

    if (distF == 0){
    distF = 100;
  }
  if (distF == 4){
    distF = 100;
  }

  if ((distR == 0) || (distR == 4)) {
    distR = 100;
  }

  
  Serial.print(distF);
  Serial.print("  ");
  Serial.println(distR);

  if ((distR > longDist) && (distF > longDist)) {
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    //Serial.println("7, 8 LOW");
  } else if ((distR > shortDist) && (distR < longDist)) {
    digitalWrite(8, HIGH);
    //Serial.println("8 HIGH");
  } else {
    digitalWrite(7, HIGH);
    //Serial.println("7 HIGH");
  }

  if ((distF > longDist) && (distR > longDist)) {
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    //Serial.println("7, 8 LOW");
  } else if ((distF > shortDist) && (distF < longDist)) {
    digitalWrite(8, HIGH);
    //Serial.println("8 HIGH");
  } else {
    digitalWrite(7, HIGH);
    //Serial.println("7 HIGH");
  }

  delay(50);
}

