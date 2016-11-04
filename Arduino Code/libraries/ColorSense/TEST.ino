#include <ColorSense.h>

ColorSense kokoti(2,3,4);
void setup() {
 
  //begin serial communication
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
    kokoti.checkBalance();
    kokoti.checkColour();
    Serial.println(kokoti.getRedValue());
        Serial.println(kokoti.getGreenValue());
            Serial.println(kokoti.getBlueValue());
}
