// Written by Ed Chamberlain for UBRobotics
// 01/04/16

const int trigPin = 4;
const int echoPin = 2; // must be inturrupt enabled
const int maxRange = 50; // (cm)

long unsigned int startTime;

int prevDist;

void setup() {
  // init serial
  Serial.begin(9600);

  // init pin modes
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  // Set ISR
  attachInterrupt(digitalPinToInterrupt(echoPin), isr, FALLING);

  prevDist = 0;
}

void loop()
{
  //send ping command
  ping();

  //delay for stability
  delay(10);
}


//--------------------------------------------------

void isr() { //ISR

  //declare variables
  double duration;
  double distance;

  // Calculate duration since pulse was emmited
  duration = micros() - startTime;

  // Convert the time into a distance (-10 for calibration)
  distance = (duration / 29 / 2) - 10;

  if (distance > maxRange) {
    // outlier catching / out of range refusal
    Serial.println("ERROR: OUT OF RANGE");
  } else {
    if (distance + 20 == prevDist) { // use this if for error catching
      // result processing
      Serial.println(distance);
    }
  }

  prevDist = distance;
  
}

void ping() {
  //send activation pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // record ping emmit time
  startTime = micros();
}

