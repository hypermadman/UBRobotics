// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%% Quadrature Decoder for EMG30 Motors %%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// © UBRobotics 2016
// Written by Ed Chamberlain
//
// This sketch will read the encoder value for a sinle EMG30 motors
// and print the current count to the serial interface.
//
// Do NOT accidently supply 12V to the Arduino - you will break it.
// You can use the 5V supply for the encoders, remeber to connect
// grounds. Pull encoder signals high using 10K resistors. Internal
// pull-ups are used in this sketch but wil vary by board. Do not
// rely on these for extensive testing.
//
// EMG30 Wiring as follows:
//   Wire colour |  Connection
//   ------------|------------------
//    1 Purple   |  Hall Sensor B Vout
//    2 Blue     |  Hall sensor A Vout
//    3 Green    |  Hall sensor ground
//    4 Brown    |  Hall sensor Vcc
//    5 Red      |  + Motor
//    6 Black    |  - Motor

// ----------------------------------------------------------------
// ----------------------- Global Variables -----------------------
// ----------------------------------------------------------------

//pulse counter
volatile  long count = 0;

#define intMode CHANGE

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

void setup() {
  // init pin modes, active pullups
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  // init serial
  Serial.begin(250000);

  // init interrupts
  attachInterrupt(7, encoderA, intMode);
  attachInterrupt(8, encoderB, intMode) ;
}

void loop() {
  // put your main code here, to run repeatedly:
}

// ----------------------------------------------------------------
// ----------------- Interrupt Service Routines -------------------
// ----------------------------------------------------------------

void encoderA () {
  // decode quadrature
  (digitalRead(7) ^ digitalRead(8)) ? count-- : count++;

  // debug to serial
  Serial.println(count);
}

void encoderB () {
  (digitalRead(7) ^ digitalRead(8)) ? count++ : count--;
  Serial.println(count);
}

// serial event will inturrupt whenever serial data is available
void serialEvent() {
  if (Serial.read() == 'r') {
    // reset count when receiving 'r'
    count = 0;
    Serial.println(count);
  }
}

