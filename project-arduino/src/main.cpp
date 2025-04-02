#include <Arduino.h>
#include <L293D.h>
#include <Servo.h>

// put function declarations here:
void processCommand(String command);
void fireSequence();
void pollReload();

// declare variables
Servo swivelServo;
Servo verticalSwivelServo;
Servo cameraServo;
Servo triggerServo;
int swivelPin = 3;
int verticalSwivelPin = 6;
int cameraPin = 9;
int triggerPin = 5;
int flywheelPin = 7;
int swivelPos = 100;
int prevSwivelPos = 100;
int swivelOffset = 4;  // offset between gun and camera
int verticalSwivelPos = 90;
int prevVerticalSwivelPos = 90;
const int VRxPin = A0;
const int VRyPin = A1;
const int SWPin = 4;
const int ledPin = 2;
int loaded = 0;
const int buttonPin = 12;

void setup() {
  // put your setup code here, to run once:
  // begin serial communication
  Serial.begin(9600);

  // setup flywheel motor and spin it
  pinMode(flywheelPin, OUTPUT);
  digitalWrite(flywheelPin, HIGH);

  // setup swivel servo motor
  swivelServo.attach(swivelPin);
  swivelServo.write(swivelPos + swivelOffset);

  cameraServo.attach(cameraPin);
  cameraServo.write(swivelPos);

  verticalSwivelServo.attach(verticalSwivelPin);
  verticalSwivelServo.write(verticalSwivelPos);

  // setup trigger servo motor
  triggerServo.attach(triggerPin);
  triggerServo.write(60);

  // SW pin
  pinMode(SWPin, INPUT_PULLUP);

  // LED ready-to-fire pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // reload button
  pinMode(buttonPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(12);

  // poll reload button
  pollReload();

  // process serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // return vertical state information if command was vertical
    if (command == "up" || command == "down") {
      Serial.print("VerticalState: ");
      Serial.println(verticalSwivelPos);
    } else {  // respond with what was received
      Serial.print("Received: ");
      Serial.println(command);
    }

    processCommand(command);
  }

  // control the swivel
  swivelPos += (int)((analogRead(VRyPin) - 513) / 500);
  verticalSwivelPos -= (int)((analogRead(VRxPin) - 513) / 500);

  if (swivelPos < 40) {  // restrict within range
    swivelPos = 40;
  } else if (swivelPos > 140) {
    swivelPos = 140;
  }
  if (verticalSwivelPos < 60) {  // restrict within range
    verticalSwivelPos = 60;
  } else if (verticalSwivelPos > 100) {
    verticalSwivelPos = 100;
  }

  // update if a position has been changed
  if (swivelPos != prevSwivelPos) {
    swivelServo.write(swivelPos + swivelOffset);
    cameraServo.write(swivelPos);
    prevSwivelPos = swivelPos;
  }
  if (verticalSwivelPos != prevVerticalSwivelPos) {
    verticalSwivelServo.write(verticalSwivelPos);
    prevVerticalSwivelPos = verticalSwivelPos;
  }

  // control the trigger
  if (digitalRead(SWPin) == LOW) {
    fireSequence();
  }
}

// put function definitions here:
void processCommand(String command) {
  command.toLowerCase();
  int step = 1;

  if (command == "left") {
    swivelPos += step;
  } else if (command == "right") {
    swivelPos -= step;
  } else if (command == "up") {
    verticalSwivelPos -= step;
  } else if (command == "down") {
    verticalSwivelPos += step;
  } else if (command == "fire") {
    fireSequence();
  }
}

void fireSequence() {
  // if the turret is not loaded, return
  if (loaded == 0) {
    return;
  }

  // fire the rubber band
  triggerServo.write(150);
  delay(300);
  triggerServo.write(60);

  // update loaded state and associated LED
  digitalWrite(ledPin, LOW);
  loaded = 0;
}

void pollReload() {
  int reading = digitalRead(buttonPin);
  if (reading == HIGH) {  // update loaded state and associated LED
    digitalWrite(ledPin, HIGH);
    loaded = 1;
  }
}