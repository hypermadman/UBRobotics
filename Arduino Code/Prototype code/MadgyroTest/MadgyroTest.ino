#include <Madgyro.h>


  Madgyro testo; //Reset

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
    float yaw = testo.readYaw(); //Cast the output of readYaw() to a float
    Serial.println(yaw); //Print it
}
