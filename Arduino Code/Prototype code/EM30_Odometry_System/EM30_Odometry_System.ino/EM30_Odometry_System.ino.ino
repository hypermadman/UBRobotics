// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%% Odemetry contorller for EMG30 Motors %%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// © UBRobotics 2016
// Written by Ed Chamberlain
//
// UNFINISHED
//
// This sketch will acts as a simple odometry and positioning system
// for a differential drive robot running using EMG30 (or similar)
// motors.
//
// This sketch requires an arduino board with 4+ inturrupts such as
// the 101, Mega, zero, Leonardo, Micro, Due.
//
// Do NOT accidently supply 12V to the Arduino - you will break it.
// You can use the 5V supply for the encoders, remeber to connect
// grounds. Pull encoder signals high using 4K7 - 10K resistors
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
//
// Co-ords reference centrepoint of robot and orientation
// references forward direction. Position in arena as shown below.
//
// Y
// ^
// |
// |   ^
// | I-X-I
// + - - - - > X
//

// ----------------------------------------------------------------
// ----------------------- Global Variables -----------------------
// ----------------------------------------------------------------

// ------ EDIT THESE VARIABLES ------

//inturrupt mode
#define intMode CHANGE //CHANGE, FALLING?????

// Set wheel base and wheel diamter here:
static const int wheelBase = 207 ;
static const int wheelDiam = 100;

// Set which pin the wheel encoders are connected to. These must
// be inturrpt enabled. Ensure encoders are pulled up to VCC
// using suitably selected resistors.
#define LeftEncA 2
#define LeftEncB 3
#define RightEncA 4
#define RightEncB 5

// ------ DO NOT EDIT BELOW THIS LINE -------

// define vector struct
struct vector {
  float x; // x component
  float y; // y component
  float o; // orientation component
};

// Set initial position here:
vector pos = {0, 0, 0};

#define pi 3.14159
double wheeltravelperstep;
double centerpointtravellength;

vector travelStepL, travelStepR;

//pulse counter
volatile  long countL = 0;
volatile  long countR = 0;

// ----------------------------------------------------------------
// ------------------------- Program Loop -------------------------
// ----------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Running");

  //init inputs
  pinMode(LeftEncA, INPUT_PULLUP);
  pinMode(LeftEncB, INPUT_PULLUP);
  pinMode(RightEncA, INPUT_PULLUP);
  pinMode(RightEncB, INPUT_PULLUP);

  // init inturrpts
  attachInterrupt(digitalPinToInterrupt(LeftEncA), ISRLeftEncA,
                  intMode);
  attachInterrupt(digitalPinToInterrupt(LeftEncB), ISRLeftEncB,
                  intMode);
  attachInterrupt(digitalPinToInterrupt(RightEncA), ISRRightEncA,
                  intMode);
  attachInterrupt(digitalPinToInterrupt(RightEncB), ISRRightEncB,
                  intMode);
  Serial.println("SETUP: ISR init");

  //compute constant global variables based on robot dynamics
  float wheeltravelperstep = wheelDiam * pi / 360;
  float centerpointtravellength = wheeltravelperstep / 2;
  double pheta = 2 * asin((wheeltravelperstep / 2) / (wheelBase));

  travelStepL.y = (wheelBase / 2) * sin(pheta);
  travelStepL.x = (wheelBase / 2) - (wheelBase / 2) * cos(pheta);
  travelStepL.o = -pheta;

  travelStepR.y = travelStepL.y;
  travelStepR.x = -travelStepL.x;
  travelStepL.o = pheta;
}

void loop() {
  Serial.print(countL);
  Serial.print("  ");
  Serial.print(countR);
  Serial.print("  | . ");
  Serial.print(pos.x);
  Serial.print("  ");
  Serial.print(pos.y);
  Serial.print("  ");
  Serial.println(pos.o);

  delay(100);
}

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

float deg2rad (float deg) {
  float rad = (deg / 360) * (2 * pi);
  return rad;
}

float rad2deg (float rad) {
  float deg = (rad / (2 * pi)) * (360);
  return deg;
}

vector rotateVector(vector input, float rotation) {
  vector output;

  output.x = input.x * cos(deg2rad(rotation)) - input.y *
             sin(deg2rad(rotation));
  output.y = input.y * sin(deg2rad(rotation)) - input.x *
             cos(deg2rad(rotation));
  output.o = rotation;

  return output;
}

vector addVector(vector input1, vector input2) {
  vector output;

  output.x = input1.x + input2.x;
  output.y = input1.y + input2.y;
  output.o = input1.o + input2.o;

  return output;
}

// ----------------------------------------------------------------
// ----------------- Interrupt Service Routines -------------------
// ----------------------------------------------------------------

void ISRLeftEncA() {
  (digitalRead(LeftEncA) ^ digitalRead(LeftEncB)) ? countL-- : countL++;
  /*
    if (digitalRead(LeftEncA) ^ digitalRead(LeftEncB)) {
      pos = addVector(pos, rotateVector(travelStepL, pos.o + pi));
    } else {
      pos = addVector(pos, rotateVector(travelStepL, pos.o));
    }
  */
}

void ISRLeftEncB() {
  (digitalRead(LeftEncA) ^ digitalRead(LeftEncB)) ? countL++ : countL--;

  /*
    if (digitalRead(LeftEncA) ^ digitalRead(LeftEncB)) {
      pos = addVector(pos, rotateVector(travelStepL, pos.o));
    } else {
      pos = addVector(pos, rotateVector(travelStepL, pos.o + pi));
    }
  */
}

void ISRRightEncA() {
  (digitalRead(RightEncA) ^ digitalRead(RightEncB)) ? countR-- : countR++;

  /*
    if (digitalRead(LeftEncA) ^ digitalRead(LeftEncB)) {
      pos = addVector(pos, rotateVector(travelStepR, pos.o + pi));
    } else {
      pos = addVector(pos, rotateVector(travelStepR, pos.o));
    }
  */
}

void ISRRightEncB() {
  (digitalRead(RightEncA) ^ digitalRead(RightEncB)) ? countR++ : countR--;

  /*
    if (digitalRead(LeftEncA) ^ digitalRead(LeftEncB)) {
      pos = addVector(pos, rotateVector(travelStepR, pos.o));
    } else {
      pos = addVector(pos, rotateVector(travelStepR, pos.o + pi));
    }
  */
}
