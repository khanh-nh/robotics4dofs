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
extern int normalStepDelayMs;

enum ToolType {
  TOOL_NONE = -1,
  TOOL_GRIPPER = 0,
  TOOL_HAND = 1,
  TOOL_DRILL = 2
};

extern ToolType activeTool;
extern int currentToolStationSlot;

void setupServos();
void setupStepper();
void setupRelays();
void moveStepper(float degrees);
void rotateToolStationToSlot(int targetSlot);
bool setActiveTool(ToolType tool);
void setMotionSpeed(int speedMs);
const char *toolName(ToolType tool);
int toolSlot(ToolType tool);
float clampJoint(int jointID, float value);
int clampServoAngle(int servoID, int angle);

void printCurrentServoAngles();
void testArmServos();
void moveToRestPose();
void moveAllSlow(int target0, int target1, int target2, int target3);
void moveJointAngles(float pitch, float roll, float yaw, float elbow);
void moveServoSlow(Servo &servo, int &currentAngle, int targetAngle, int servoID);

void setMagnet(bool enabled);
void setToolPower(bool enabled);
void printToolStatus();
void moveToToolChangePose();
void setToolChangePose(float pitch, float roll, float yaw, float elbow);
void printToolChangePose();
void moveToToolPrechangePose();
void moveIntoToolDockSlow();
void shakeToolDockPose();
void moveOutFromDockWithTool();
void moveOutFromDockNoTool();
bool pickupTool(ToolType newTool);
bool removeHeldTool();
bool changeHeldTool(ToolType newTool);
void testPickedTool(ToolType tool);
void showcaseTools();
void gripperOpen();
void gripperClose();
void toolMotion();
void moveToXYZ(Vec3 target);
