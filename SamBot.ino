#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <AFMotor.h> // V1

// SamBot.ino
// Author: Samuel Haws (https://github.com/SamuelHaws)

////////////////////////////////////////////////////////////////////////
/////////////////// User Configuration Variables ///////////////////////
////////////////////////////////////////////////////////////////////////
const int redLEDPin=48;
const int dfBusyPin=32; // DFPlayer busy pin: low == playing audio, high == not
const int usTrigPin=36; // HC-SR04 Ultrasonic Sensor
const int usEchoPin=38;

const int frMotorCode=1; // "Front right", value is motor identification on drive shield
const int flMotorCode=2;
const int blMotorCode=3;
const int brMotorCode=4;

const int introAudioIdx=3; // Index of file for DFPlayer
const int obstacleAudioIdx=2;
const int wowAudioIdx=1;
const int volumeLevel=23; // 0 - 30
const int obstacleTriggerDistance=17; // cm
const int maxTriggerCount=14; // Number of sequential "too close" distance reads to trigger obstacle
const int flashLEDDelay=75; // ms
const int obstacleLEDTime=2000; // Duration of flashing when obstacle encountered (ms)
const int obstacleBackwardTime=250; // Duration of backward motion when obstacle encountered (ms)
const int spinTime=2000; // ms
////////////////////////////////////////////////////////////////////////
/////////////////// End Configuration Variables ////////////////////////
////////////////////////////////////////////////////////////////////////

int triggerCount=0; // Current number of sequential distances below trigger
const float speedOfSoundCoeff=0.034; // cm per microsecond
float pingTravelTime; // microseconds
float distance; // Between sensor and object in front, in cm
char btSignal; // Signal received from HC-05 Bluetooth Module

// DFPlayer sends audio from files on SD Card to speaker
DFRobotDFPlayerMini dfPlayer; 

// Motors (front-right, front-left, etc.)
AF_DCMotor frMotor(frMotorCode);
AF_DCMotor flMotor(flMotorCode);
AF_DCMotor blMotor(blMotorCode);
AF_DCMotor brMotor(brMotorCode);

void setup() {
  pinMode(redLEDPin,OUTPUT);
  pinMode(usTrigPin,OUTPUT);
  pinMode(usEchoPin,INPUT);
  pinMode(dfBusyPin,INPUT);
  
  // Serial for computer (printing/debugging)
  Serial.begin(115200);
  // Serial for HC-05 Bluetooth Module
  Serial1.begin(9600);
  // Serial for DFPlayer
  Serial2.begin(9600);

  if (!dfPlayer.begin(Serial2)) {  
    Serial.println(F("Unable to begin DFPlayer Mini..."));
    Serial.println(F("Please recheck the connections and SD card."));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  
  dfPlayer.volume(volumeLevel);
}

void loop() {
  triggerCount = 0;
  if(Serial1.available() > 0){
    btSignal = Serial1.read();
    switch(btSignal){
      case 'F':
        motorForward();
        break;
      case 'B':
        motorBackward();
        break;
      case 'L':
        motorLeft();
        break;
      case 'R':
        motorRight();
        break;
      case 'V':
        // If DFPlayer not currently playing audio, play intro audio
        if (digitalRead(dfBusyPin) == 1) {
          dfPlayer.play(introAudioIdx);
        }
        break;
      case 'X':
        wow();
      default:
        motorStop();
        break;
    }
  }

  // Sense/calculate distance
  digitalWrite(usTrigPin,LOW);
  delayMicroseconds(10);
  digitalWrite(usTrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrigPin,LOW);
  pingTravelTime=pulseIn(usEchoPin,HIGH);
  // Must divide by two to get distance, because the signal goes out and back
  distance = (pingTravelTime*speedOfSoundCoeff)/float(2);
  // If detect some number of sequential "too close" distances, handle obstacle
  while(distance < obstacleTriggerDistance) {
    triggerCount++;
    if (triggerCount >= maxTriggerCount) {
      handleObstacle();
      break;
    }
  }
}

// Action to take when obstacle detected by ultrasonic sensor
void handleObstacle() {
  // If DFPlayer not currently playing audio, play obstacle audio
  if (digitalRead(dfBusyPin) == 1) {
    dfPlayer.play(obstacleAudioIdx);
  }
  // Move backward slightly and Flash LED for short duration
  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  unsigned long elapsedTime;
  motorBackward();
  while (true) {
    endTime = millis();
    elapsedTime = endTime - startTime;
    // Flash LED for some duration
    if (elapsedTime <= obstacleLEDTime) {
      flashLED();
    }
    // End backward motion after some duration
    if(elapsedTime > obstacleBackwardTime) {
      motorStop();
    }
    // Control is returned to user once both obstacleTimes are elapsed
    if (elapsedTime > max(obstacleLEDTime, obstacleBackwardTime)) {
      break;
    }
  }
}

void flashLED() {
  digitalWrite(redLEDPin,HIGH);
  delay(flashLEDDelay);
  digitalWrite(redLEDPin,LOW);
  delay(flashLEDDelay);
}

// Motor movement functions

void motorForward() {
  flMotor.setSpeed(255);
  flMotor.run(FORWARD);
  frMotor.setSpeed(255);
  frMotor.run(FORWARD);
  blMotor.setSpeed(255);
  blMotor.run(FORWARD);
  brMotor.setSpeed(255);
  brMotor.run(FORWARD);
}
void motorBackward() {
  flMotor.setSpeed(255);
  flMotor.run(BACKWARD);
  frMotor.setSpeed(255);
  frMotor.run(BACKWARD);
  blMotor.setSpeed(255);
  blMotor.run(BACKWARD);
  brMotor.setSpeed(255);
  brMotor.run(BACKWARD);
}
void motorLeft() {
  flMotor.setSpeed(255);
  flMotor.run(BACKWARD);
  frMotor.setSpeed(255);
  frMotor.run(FORWARD);
  blMotor.setSpeed(255);
  blMotor.run(BACKWARD);
  brMotor.setSpeed(255);
  brMotor.run(FORWARD);
}
void motorRight() {
  flMotor.setSpeed(255);
  flMotor.run(FORWARD);
  frMotor.setSpeed(255);
  frMotor.run(BACKWARD);
  blMotor.setSpeed(255);
  blMotor.run(FORWARD);
  brMotor.setSpeed(255);
  brMotor.run(BACKWARD);
}
void motorStop() {
  flMotor.setSpeed(0);
  frMotor.setSpeed(0);
  blMotor.setSpeed(0);
  brMotor.setSpeed(0);
}







// Easter egg... spin in a circle and play anime "wow" meme audio
void wow() {
  // If DFPlayer not currently playing audio, play wow audio
  if (digitalRead(dfBusyPin) == 1) {
    dfPlayer.play(wowAudioIdx);
  }
  motorRight();
  delay(spinTime);
  motorStop();
  delay(500);
  motorLeft();
  delay(spinTime);
  motorStop();
}
