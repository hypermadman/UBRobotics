const int buttonPin = 12;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
bool led = false; //IS LED ON

void setup() {
  attachInterrupt(12,changeState,RISING);
}

void loop() {

}

void changeState(){
  switch (led){
    case false:
      led = true;
      tone(ledPin,10);
      break;
    case true: 
      led = false;
      noTone(ledPin);
      break;
  }
}

