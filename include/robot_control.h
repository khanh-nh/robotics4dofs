#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "kinematics.h"

extern Servo servo0;
extern Servo servo1;
extern Servo servo2;
extern Servo servo3;
extern Servo toolServo;

extern int servo0Angle;
extern int servo1Angle;
extern int servo2Angle;
extern int servo3Angle;

extern float currentPitch;
extern float currentRoll;
extern float currentYaw;
extern float currentElbow;

extern int stepDelayMs;

void setupServos();
void setupStepper();
void moveStepper(float degrees);
float clampJoint(int jointID, float value);
int clampServoAngle(int servoID, int angle);

void printCurrentServoAngles();
void moveToRestPose();
void moveAllSlow(int target0, int target1, int target2, int target3);
void moveJointAngles(float pitch, float roll, float yaw, float elbow);
void moveServoSlow(Servo &servo, int &currentAngle, int targetAngle, int servoID);

void toolMotion();
void moveToXYZ(Vec3 target);