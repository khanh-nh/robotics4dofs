#include "robot_control.h"
#include "config.h"

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo toolServo;

int servo0Angle = servo0Rest;
int servo1Angle = servo1Rest;
int servo2Angle = servo2Rest;
int servo3Angle = servo3Rest;

float currentPitch = 0;
float currentRoll  = 0;
float currentYaw   = 0;
float currentElbow = 0;

int stepDelayMs = 15;
ToolType activeTool = TOOL_NONE;
int currentToolStationSlot = toolSlotGripper;
float currentToolChangePitch = toolChangePitch;
float currentToolChangeRoll = toolChangeRoll;
float currentToolChangeYaw = toolChangeYaw;
float currentToolChangeElbow = toolChangeElbow;

static long smoothStepPermille(int step, int totalSteps) {
  long t = ((long)step * 1000L) / totalSteps;
  return (3L * t * t - 2L * t * t * t / 1000L) / 1000L;
}

static int interpolateAngle(int startAngle, int targetAngle, long progressPermille) {
  long delta = targetAngle - startAngle;
  long offset = (delta * progressPermille + (delta >= 0 ? 500L : -500L)) / 1000L;
  return startAngle + offset;
}

void setupServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo0.setPeriodHertz(50);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  toolServo.setPeriodHertz(50);

  servo0.attach(servo0Pin, minPulse, maxPulse);
  servo1.attach(servo1Pin, minPulse, maxPulse);
  servo2.attach(servo2Pin, minPulse, maxPulse);
  servo3.attach(servo3Pin, minPulse, maxPulse);
  toolServo.attach(toolServoPin, minPulse, maxPulse);

  servo0.write(servo0Rest);
  servo1.write(servo1Rest);
  servo2.write(servo2Rest);
  servo3.write(servo3Rest);
  toolServo.write(toolServoRestAngle);

  servo0Angle = servo0Rest;
  servo1Angle = servo1Rest;
  servo2Angle = servo2Rest;
  servo3Angle = servo3Rest;

  currentPitch = 0;
  currentRoll  = 0;
  currentYaw   = 0;
  currentElbow = 0;
}

void setupStepper() {
  pinMode(stepperStepPin, OUTPUT);
  pinMode(stepperDirPin, OUTPUT);
  digitalWrite(stepperStepPin, LOW);
  digitalWrite(stepperDirPin, LOW);
}

void setupRelays() {
  pinMode(magnetRelayPin, OUTPUT);
  pinMode(toolPowerRelayPin, OUTPUT);
  setMagnet(false);
  setToolPower(false);
}

void moveStepper(float degrees) {
  if (degrees == 0) return;

  Serial.print("Moving stepper ");
  Serial.print(degrees);
  Serial.println(" degrees.");

  if (degrees > 0) {
    digitalWrite(stepperDirPin, HIGH); // Adjust HIGH/LOW based on actual desired direction
  } else {
    digitalWrite(stepperDirPin, LOW);
  }

  float absoluteDegrees = abs(degrees);
  int steps = round((absoluteDegrees / 360.0) * stepperStepsPerRev);

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepperStepPin, HIGH);
    delayMicroseconds(1000); // 1ms pulse
    digitalWrite(stepperStepPin, LOW);
    delayMicroseconds(1000); // 1ms pause, so 2ms per step
  }
  Serial.println("Stepper motion done.");
}

void rotateToolStationToSlot(int targetSlot) {
  targetSlot = ((targetSlot % toolStationSlots) + toolStationSlots) % toolStationSlots;

  int slotDelta = targetSlot - currentToolStationSlot;
  if (slotDelta > toolStationSlots / 2) slotDelta -= toolStationSlots;
  if (slotDelta < -(toolStationSlots / 2)) slotDelta += toolStationSlots;

  if (slotDelta == 0) {
    Serial.print("Tool station already at slot ");
    Serial.println(currentToolStationSlot);
    return;
  }

  float degrees = slotDelta * toolStationStepDeg;
  Serial.print("Rotating tool station from slot ");
  Serial.print(currentToolStationSlot);
  Serial.print(" to slot ");
  Serial.print(targetSlot);
  Serial.print(" (");
  Serial.print(degrees);
  Serial.println(" deg).");

  moveStepper(degrees);
  currentToolStationSlot = targetSlot;
}

const char *toolName(ToolType tool) {
  if (tool == TOOL_GRIPPER) return "gripper";
  if (tool == TOOL_VACUUM) return "vacuum";
  if (tool == TOOL_DRILL) return "drill";
  return "none";
}

int toolSlot(ToolType tool) {
  if (tool == TOOL_GRIPPER) return toolSlotGripper;
  if (tool == TOOL_VACUUM) return toolSlotVacuum;
  if (tool == TOOL_DRILL) return toolSlotDrill;
  return -1;
}

bool setActiveTool(ToolType tool) {
  if (tool != TOOL_NONE && tool != TOOL_GRIPPER && tool != TOOL_VACUUM && tool != TOOL_DRILL) {
    Serial.println("Invalid tool.");
    return false;
  }

  activeTool = tool;

  if (activeTool == TOOL_NONE) {
    setToolPower(false);
  } else {
    setToolPower(true);
  }

  Serial.print("Active tool set to: ");
  Serial.println(toolName(activeTool));
  return true;
}

void moveToToolChangePose() {
  Serial.println("Moving to tool exchange pose...");
  moveJointAngles(currentToolChangePitch, currentToolChangeRoll, currentToolChangeYaw, currentToolChangeElbow);
  delay(toolStationSettleMs);
}

static void moveJointAnglesWithDelay(float pitch, float roll, float yaw, float elbow, int temporaryDelayMs) {
  int previousDelayMs = stepDelayMs;
  stepDelayMs = max(stepDelayMs, temporaryDelayMs);
  moveJointAngles(pitch, roll, yaw, elbow);
  stepDelayMs = previousDelayMs;
}

void moveToToolPrechangePose() {
  Serial.println("Moving to tool prechange pose...");
  moveJointAngles(
    currentToolChangePitch,
    currentToolChangeRoll,
    currentToolChangeYaw,
    clampJoint(3, currentToolChangeElbow + toolPrechangeElbowOffset)
  );
  delay(toolStationSettleMs);
}

void moveIntoToolDockSlow() {
  Serial.println("Docking slowly from prechange to tool exchange pose...");
  moveJointAnglesWithDelay(
    currentToolChangePitch,
    currentToolChangeRoll,
    currentToolChangeYaw,
    currentToolChangeElbow,
    toolDockSlowDelayMs
  );
  delay(toolStationSettleMs);
}

void shakeToolDockPose() {
  Serial.println("Searching magnet pose with slow sequential shake...");

  float basePitch = currentToolChangePitch;
  float baseRoll = currentToolChangeRoll;
  float baseYaw = currentToolChangeYaw;
  float baseElbow = currentToolChangeElbow;
  float shake = toolDockShakeDeg;

  moveJointAnglesWithDelay(clampJoint(0, basePitch + shake), baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(clampJoint(0, basePitch - shake), baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);

  moveJointAnglesWithDelay(basePitch, clampJoint(1, baseRoll + shake), baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, clampJoint(1, baseRoll - shake), baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);

  moveJointAnglesWithDelay(basePitch, baseRoll, clampJoint(2, baseYaw + shake), baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, clampJoint(2, baseYaw - shake), baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);

  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, clampJoint(3, baseElbow + shake), toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, clampJoint(3, baseElbow - shake), toolDockSlowDelayMs);
  moveJointAnglesWithDelay(basePitch, baseRoll, baseYaw, baseElbow, toolDockSlowDelayMs);
}

void moveOutFromDockWithTool() {
  Serial.println("Moving out from dock with tool...");
  moveJointAnglesWithDelay(
    clampJoint(0, currentToolChangePitch + toolMoveOutPitchOffset),
    currentToolChangeRoll,
    currentToolChangeYaw,
    clampJoint(3, currentToolChangeElbow + toolMoveOutElbowWithToolOffset),
    toolDockSlowDelayMs
  );
  delay(toolStationSettleMs);
}

void moveOutFromDockNoTool() {
  Serial.println("Moving out from dock with no tool...");
  moveJointAnglesWithDelay(
    clampJoint(0, currentToolChangePitch + toolMoveOutPitchOffset),
    currentToolChangeRoll,
    currentToolChangeYaw,
    clampJoint(3, currentToolChangeElbow + toolMoveOutElbowNoToolOffset),
    toolDockSlowDelayMs
  );
  delay(toolStationSettleMs);
  moveToToolPrechangePose();
}

void setToolChangePose(float pitch, float roll, float yaw, float elbow) {
  currentToolChangePitch = clampJoint(0, pitch);
  currentToolChangeRoll = clampJoint(1, roll);
  currentToolChangeYaw = clampJoint(2, yaw);
  currentToolChangeElbow = clampJoint(3, elbow);

  Serial.println("Updated tool exchange pose.");
  printToolChangePose();
}

void printToolChangePose() {
  Serial.print("Tool exchange pose q = [");
  Serial.print(currentToolChangePitch, 1);
  Serial.print(", ");
  Serial.print(currentToolChangeRoll, 1);
  Serial.print(", ");
  Serial.print(currentToolChangeYaw, 1);
  Serial.print(", ");
  Serial.print(currentToolChangeElbow, 1);
  Serial.println("] deg");
}

bool pickupTool(ToolType newTool) {
  int newSlot = toolSlot(newTool);
  if (newSlot < 0) {
    Serial.println("Invalid pickup tool.");
    return false;
  }

  if (activeTool != TOOL_NONE) {
    Serial.print("Cannot pickup ");
    Serial.print(toolName(newTool));
    Serial.print(" while holding ");
    Serial.print(toolName(activeTool));
    Serial.println(". Use change/remove first.");
    return false;
  }

  Serial.print("Initial pickup: none -> ");
  Serial.println(toolName(newTool));

  rotateToolStationToSlot(newSlot);
  moveToToolPrechangePose();
  moveIntoToolDockSlow();
  shakeToolDockPose();
  delay(toolStationSettleMs);
  setMagnet(true);
  delay(toolMagnetSettleMs);
  setActiveTool(newTool);
  moveOutFromDockWithTool();
  printToolStatus();
  return true;
}

bool removeHeldTool() {
  if (activeTool == TOOL_NONE) {
    Serial.println("No tool is currently held.");
    return true;
  }

  ToolType oldTool = activeTool;
  int oldSlot = toolSlot(oldTool);

  Serial.print("Removing held tool: ");
  Serial.println(toolName(oldTool));

  moveToToolPrechangePose();
  rotateToolStationToSlot(oldSlot);
  moveIntoToolDockSlow();
  setToolPower(false);
  delay(toolStationSettleMs);
  setMagnet(false);
  delay(toolMagnetSettleMs);
  setActiveTool(TOOL_NONE);
  moveOutFromDockNoTool();
  printToolStatus();
  return true;
}

bool changeHeldTool(ToolType newTool) {
  int newSlot = toolSlot(newTool);
  if (newSlot < 0) {
    Serial.println("Invalid target tool.");
    return false;
  }

  if (activeTool == TOOL_NONE) {
    return pickupTool(newTool);
  }

  if (activeTool == newTool) {
    Serial.print("Already holding ");
    Serial.println(toolName(newTool));
    printToolStatus();
    return true;
  }

  ToolType oldTool = activeTool;
  int oldSlot = toolSlot(oldTool);

  Serial.print("Changing tool: ");
  Serial.print(toolName(oldTool));
  Serial.print(" -> ");
  Serial.println(toolName(newTool));

  moveToToolPrechangePose();
  rotateToolStationToSlot(oldSlot);
  moveIntoToolDockSlow();
  setToolPower(false);
  delay(toolStationSettleMs);
  setMagnet(false);
  delay(toolMagnetSettleMs);
  activeTool = TOOL_NONE;
  moveOutFromDockNoTool();

  rotateToolStationToSlot(newSlot);
  moveToToolPrechangePose();
  moveIntoToolDockSlow();
  shakeToolDockPose();
  delay(toolStationSettleMs);
  setMagnet(true);
  delay(toolMagnetSettleMs);
  setActiveTool(newTool);
  moveOutFromDockWithTool();
  printToolStatus();
  return true;
}

float clampJoint(int jointID, float value) {
  if (jointID == 0) return constrain(value, pitchMin, pitchMax);
  if (jointID == 1) return constrain(value, rollMin, rollMax);
  if (jointID == 2) return constrain(value, yawMin, yawMax);
  if (jointID == 3) return constrain(value, elbowMin, elbowMax);
  return value;
}

int clampServoAngle(int servoID, int angle) {
  if (servoID == 0) return constrain(angle, servo0Min, servo0Max);
  if (servoID == 1) return constrain(angle, servo1Min, servo1Max);
  if (servoID == 2) return constrain(angle, servo2Min, servo2Max);
  if (servoID == 3) return constrain(angle, servo3Min, servo3Max);
  return constrain(angle, 0, 180);
}

void moveServoSlow(Servo &servo, int &currentAngle, int targetAngle, int servoID) {
  targetAngle = clampServoAngle(servoID, targetAngle);

  int startAngle = currentAngle;
  int steps = abs(targetAngle - startAngle);

  if (steps == 0) {
    return;
  }

  for (int step = 1; step <= steps; step++) {
    long progress = smoothStepPermille(step, steps);
    currentAngle = interpolateAngle(startAngle, targetAngle, progress);
    servo.write(currentAngle);
    delay(stepDelayMs);
  }

  currentAngle = targetAngle;
  servo.write(currentAngle);
}

void moveAllSlow(int target0, int target1, int target2, int target3) {
  target0 = clampServoAngle(0, target0);
  target1 = clampServoAngle(1, target1);
  target2 = clampServoAngle(2, target2);
  target3 = clampServoAngle(3, target3);

  int start0 = servo0Angle;
  int start1 = servo1Angle;
  int start2 = servo2Angle;
  int start3 = servo3Angle;

  int maxDelta = max(
    max(abs(target0 - start0), abs(target1 - start1)),
    max(abs(target2 - start2), abs(target3 - start3))
  );

  if (maxDelta == 0) {
    printCurrentServoAngles();
    return;
  }

  for (int step = 1; step <= maxDelta; step++) {
    long progress = smoothStepPermille(step, maxDelta);

    servo0Angle = interpolateAngle(start0, target0, progress);
    servo1Angle = interpolateAngle(start1, target1, progress);
    servo2Angle = interpolateAngle(start2, target2, progress);
    servo3Angle = interpolateAngle(start3, target3, progress);

    servo0.write(servo0Angle);
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);

    delay(stepDelayMs);
  }

  servo0Angle = target0;
  servo1Angle = target1;
  servo2Angle = target2;
  servo3Angle = target3;

  servo0.write(servo0Angle);
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
  servo3.write(servo3Angle);

  printCurrentServoAngles();
}

void moveJointAngles(float pitch, float roll, float yaw, float elbow) {
  pitch = clampJoint(0, pitch);
  roll  = clampJoint(1, roll);
  yaw   = clampJoint(2, yaw);
  elbow = clampJoint(3, elbow);

  int target0 = round(servo0Rest + servo0Dir * pitch);
  int target1 = round(servo1Rest + servo1Dir * roll);
  int target2 = round(servo2Rest + servo2Dir * yaw);
  int target3 = round(servo3Rest + servo3Dir * elbow);

  Serial.println();
  Serial.print("Moving to q = [");
  Serial.print(pitch, 1);
  Serial.print(", ");
  Serial.print(roll, 1);
  Serial.print(", ");
  Serial.print(yaw, 1);
  Serial.print(", ");
  Serial.print(elbow, 1);
  Serial.println("] deg");

  moveAllSlow(target0, target1, target2, target3);

  currentPitch = pitch;
  currentRoll  = roll;
  currentYaw   = yaw;
  currentElbow = elbow;

  printFK(pitch, roll, yaw, elbow);
}

void moveToRestPose() {
  Serial.println("Moving to rest pose...");
  moveAllSlow(servo0Rest, servo1Rest, servo2Rest, servo3Rest);

  currentPitch = 0;
  currentRoll  = 0;
  currentYaw   = 0;
  currentElbow = 0;

  printFK(0, 0, 0, 0);
  Serial.println("Rest pose done.");
}

void printCurrentServoAngles() {
  Serial.print("Raw servo angles: ");
  Serial.print("S0 = "); Serial.print(servo0Angle);
  Serial.print(", S1 = "); Serial.print(servo1Angle);
  Serial.print(", S2 = "); Serial.print(servo2Angle);
  Serial.print(", S3 = "); Serial.println(servo3Angle);
}

void setMagnet(bool enabled) {
  digitalWrite(magnetRelayPin, enabled ? HIGH : LOW);
  Serial.print("Magnet relay 1 is ");
  Serial.println(enabled ? "ON" : "OFF");
}

void setToolPower(bool enabled) {
  digitalWrite(toolPowerRelayPin, enabled ? HIGH : LOW);
  Serial.print("Tool power relay 2 is ");
  Serial.println(enabled ? "ON" : "OFF");
}

void printToolStatus() {
  Serial.print("Active tool: ");
  Serial.println(toolName(activeTool));
  Serial.print("Tool station slot: ");
  Serial.println(currentToolStationSlot);
  Serial.println("Slots: 0=gripper, 1=vacuum, 2=drill");
}

void gripperOpen() {
  if (activeTool != TOOL_GRIPPER) {
    Serial.println("Warning: active tool is not gripper. Powering relay 2 and sending servo signal anyway.");
  }
  setToolPower(true);
  toolServo.write(toolServoRestAngle);
  Serial.println("Gripper open/rest command sent.");
}

void gripperClose() {
  if (activeTool != TOOL_GRIPPER) {
    Serial.println("Warning: active tool is not gripper. Powering relay 2 and sending servo signal anyway.");
  }
  setToolPower(true);
  toolServo.write(toolServoRunAngle);
  Serial.println("Gripper close/run command sent.");
}

void toolMotion() {
  Serial.println("Tool motion started.");

  if (activeTool == TOOL_VACUUM || activeTool == TOOL_DRILL) {
    setToolPower(true);
    Serial.print(toolName(activeTool));
    Serial.println(" is running. Use 'toolpower off' to stop it.");
  } else {
    if (activeTool == TOOL_NONE) {
      Serial.println("No active tool selected. Running gripper servo as fallback.");
    }
    gripperClose();
    delay(toolRunTimeMs);
    gripperOpen();
    delay(300);
  }

  Serial.println("Tool motion done.");
}

void moveToXYZ(Vec3 target) {
  Serial.println();
  Serial.println("Solving IK...");
  Serial.print("Target EE position [mm]: x=");
  Serial.print(target.x, 2);
  Serial.print(", y=");
  Serial.print(target.y, 2);
  Serial.print(", z=");
  Serial.println(target.z, 2);

  float qSol[4];
  float finalError;
  bool success = solveIK(target, qSol, finalError);

  Serial.println();
  Serial.println("IK result:");
  Serial.print("pitch = "); Serial.println(qSol[0], 2);
  Serial.print("roll  = "); Serial.println(qSol[1], 2);
  Serial.print("yaw   = "); Serial.println(qSol[2], 2);
  Serial.print("elbow = "); Serial.println(qSol[3], 2);
  Serial.print("Predicted IK error = ");
  Serial.print(finalError, 2);
  Serial.println(" mm");

  Vec3 predicted = calculateFK(qSol[0], qSol[1], qSol[2], qSol[3]);
  Serial.print("Predicted EE position [mm]: x=");
  Serial.print(predicted.x, 2);
  Serial.print(", y=");
  Serial.print(predicted.y, 2);
  Serial.print(", z=");
  Serial.println(predicted.z, 2);

  if (!success) {
    Serial.println();
    Serial.println("IK warning: target may be unreachable or error is too high.");
    Serial.println("Arm will NOT move.");
    Serial.println("Try a closer target or increase IK_ACCEPT_ERROR_MM.");
    Serial.println();
    return;
  }

  Serial.println();
  Serial.println("IK accepted. Moving arm...");
  moveJointAngles(qSol[0], qSol[1], qSol[2], qSol[3]);
}
