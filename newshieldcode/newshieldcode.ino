const int enPin=8;
// front left
const int stepXPin = 2; //X.STEP
const int dirXPin = 5; // X.DIR
// front right
const int stepYPin = 3; //Y.STEP
const int dirYPin = 6; // Y.DIR
// back left
const int stepZPin = 4; //Z.STEP
const int dirZPin = 7; // Z.DIR
// back right
const int stepAPin = 12; // A.STEP
const int dirAPin = 13; //A.DIR

const int stepsPerRev=200;
int pulseWidthMicros = 20; 	// microseconds
int millisBtwnSteps = 10000; // ~30 rpm

void setup() {
 	Serial.begin(9600);
 	pinMode(enPin, OUTPUT);
 	digitalWrite(enPin, LOW);
 	pinMode(stepXPin, OUTPUT);
 	pinMode(dirXPin, OUTPUT);
  pinMode(stepYPin, OUTPUT);
 	pinMode(dirYPin, OUTPUT);
  pinMode(stepZPin, OUTPUT);
 	pinMode(dirZPin, OUTPUT);
  pinMode(stepAPin, OUTPUT);
 	pinMode(dirAPin, OUTPUT);
 	Serial.println(F("CNC Shield Initialized"));
}
void loop() {
  
 	Serial.println(F("Running clockwise"));
  digitalWrite(dirXPin, HIGH);
  digitalWrite(dirYPin, LOW);
 	digitalWrite(dirZPin, HIGH);
  digitalWrite(dirAPin, LOW); // Enables the motor to move in a particular direction
 	// Makes 200 pulses for making one full cycle rotation
 	for (int i = 0; i < stepsPerRev; i++) {
 			digitalWrite(stepXPin, HIGH);
      digitalWrite(stepYPin, HIGH);
      digitalWrite(stepZPin, HIGH);
      digitalWrite(stepAPin, HIGH);
 			delayMicroseconds(pulseWidthMicros);
 			digitalWrite(stepXPin, LOW);
      digitalWrite(stepYPin, LOW);
      digitalWrite(stepZPin, LOW);
      digitalWrite(stepAPin, LOW);
 			delayMicroseconds(millisBtwnSteps);
 	}
  
  /*
 	delay(1000); // One second delay
 	Serial.println(F("Running counter-clockwise"));
 	digitalWrite(dirPin, LOW); //Changes the rotations direction
 	// Makes 400 pulses for making two full cycle rotation
 	for (int i =
   0; i < 2*stepsPerRev; i++) {
 			digitalWrite(stepPin, HIGH);
 			delayMicroseconds(pulseWidthMicros);
 			digitalWrite(stepPin, LOW);
 			delayMicroseconds(millisBtwnSteps);
 	}
 	delay(1000);
  */
}