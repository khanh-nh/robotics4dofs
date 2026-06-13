#include "motions.h"
#include "robot_control.h"
#include <Arduino.h>

void waveMotion() {
  Serial.println("Wave motion started.");

  moveJointAngles(45, 20, 0, 100);
  delay(400);
  moveJointAngles(45, 20, 20, 100);
  delay(400);
  moveJointAngles(45, 20, -20, 100);
  delay(400);
  moveJointAngles(45, 20, 20, 100);
  delay(400);
  moveJointAngles(45, 20, -20, 100);
  delay(400);

  moveToRestPose();
  Serial.println("Wave motion done.");
}

void strokingMotion() {
  Serial.println("Forward stroking motion started.");

  moveToRestPose();
  delay(500);

  Serial.println("Closing elbow before raising arm...");
  moveJointAngles(0, 0, 0, 85);
  delay(600);

  Serial.println("Raising arm forward...");
  moveJointAngles(65, 0, 0, 85);
  delay(700);

  Serial.println("Moving to forward stroking start pose...");
  moveJointAngles(85, 0, 0, 60);
  delay(700);

  Serial.println("Stroking forward...");
  moveJointAngles(100, 0, 0, 45);
  delay(450);
  moveJointAngles(80, 0, 0, 65);
  delay(450);
  moveJointAngles(100, 0, 0, 45);
  delay(450);
  moveJointAngles(80, 0, 0, 65);
  delay(450);
  moveJointAngles(100, 0, 0, 45);
  delay(450);
  moveJointAngles(80, 0, 0, 65);
  delay(450);

  Serial.println("Closing elbow before lowering arm...");
  moveJointAngles(65, 0, 0, 85);
  delay(600);

  Serial.println("Lowering arm...");
  moveJointAngles(0, 0, 0, 85);
  delay(600);

  moveToRestPose();
  Serial.println("Forward stroking motion done.");
}

void helloMotion() {
  Serial.println("Hello motion started.");

  moveToRestPose();
  delay(400);

  Serial.println("Closing elbow before shoulder pitch...");
  moveJointAngles(0, 0, 0, 100);
  delay(600);

  Serial.println("Moving to hello pose...");
  moveJointAngles(0, 100, 90, 100);
  delay(600);

  moveJointAngles(0, 100, 90, 120);
  delay(400);
  moveJointAngles(0, 100, 90, 50);
  delay(400);
  moveJointAngles(0, 100, 90, 120);
  delay(400);
  moveJointAngles(0, 100, 90, 50);
  delay(400);

  moveToRestPose();
  Serial.println("Hello motion done.");
}

void handShakeMotion() {
  Serial.println("Handshake motion started.");

  moveToRestPose();
  delay(500);

  Serial.println("Closing elbow before raising arm...");
  moveJointAngles(0, 0, 0, 85);
  delay(600);

  Serial.println("Raising arm with elbow closed...");
  moveJointAngles(75, 20, 30, 85);
  delay(700);

  Serial.println("Moving to handshake pose...");
  moveJointAngles(95, 0, 0, 55);
  delay(700);

  Serial.println("Shaking hand...");
  moveJointAngles(105, 0, 0, 50);
  delay(300);
  moveJointAngles(85, 0, 0, 65);
  delay(300);
  moveJointAngles(105, 0, 0, 50);
  delay(300);
  moveJointAngles(85, 0, 0, 65);
  delay(300);
  moveJointAngles(105, 0, 0, 50);
  delay(300);
  moveJointAngles(85, 0, 0, 65);
  delay(300);

  Serial.println("Closing elbow before lowering arm...");
  moveJointAngles(75, 20, 30, 85);
  delay(600);

  Serial.println("Lowering arm...");
  moveJointAngles(0, 0, 0, 85);
  delay(600);

  moveToRestPose();
  Serial.println("Handshake motion done.");
}

void pickAndPlaceMotion() {
  Serial.println("Pick and place motion started.");

  moveToRestPose();
  delay(500);

  Serial.println("Moving above pick position...");
  moveJointAngles(45, 30, 20, 40);
  delay(500);

  Serial.println("Moving down to pick position...");
  moveJointAngles(65, 30, 20, 70);
  delay(700);

  Serial.println("Picking object...");
  toolMotion();
  delay(500);

  Serial.println("Lifting object...");
  moveJointAngles(40, 30, 20, 50);
  delay(700);

  Serial.println("Moving above place position...");
  moveJointAngles(40, 70, 80, 50);
  delay(800);

  Serial.println("Lowering to place position...");
  moveJointAngles(60, 70, 80, 75);
  delay(700);

  Serial.println("Placing object...");
  toolMotion();
  delay(500);

  Serial.println("Moving away from place position...");
  moveJointAngles(40, 70, 80, 50);
  delay(700);

  moveToRestPose();
  Serial.println("Pick and place motion done.");
}

void frontPickMotion() {
  Serial.println("Front table-pick motion started.");

  moveToRestPose();
  delay(500);

  Serial.println("Closing elbow before reaching forward...");
  moveJointAngles(0, 0, 0, 90);
  delay(600);

  Serial.println("Raising arm forward safely...");
  moveJointAngles(55, 0, 0, 90);
  delay(700);

  Serial.println("Approaching above the table object...");
  moveJointAngles(85, 0, 0, 65);
  delay(700);

  Serial.println("Lowering to low object on table...");
  moveJointAngles(110, 0, 0, 35);
  delay(900);

  Serial.println("Trying to pick object...");
  toolMotion();
  delay(700);

  Serial.println("Lifting object...");
  moveJointAngles(85, 0, 0, 65);
  delay(700);

  Serial.println("Retracting arm...");
  moveJointAngles(55, 0, 0, 90);
  delay(700);

  Serial.println("Lowering shoulder with elbow closed...");
  moveJointAngles(0, 0, 0, 90);
  delay(600);

  moveToRestPose();
  Serial.println("Front table-pick motion done.");
}