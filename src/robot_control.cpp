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
int normalStepDelayMs = 15;
ToolType activeTool = TOOL_NONE;
int currentToolStationSlot = toolSlotGripper;
bool toolPowerEnabled = false;
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

static bool needsReturnStationNudge(ToolType tool) {
  return tool == TOOL_DRILL || tool == TOOL_GRIPPER;
}

static void nudgeStationForToolReturn(ToolType tool, bool beforeDocking) {
  float nudgeDegrees = beforeDocking ? toolReturnStationNudgeDeg : -toolReturnStationNudgeDeg;

  Serial.print(beforeDocking ? "Nudging station for tool return: " : "Rolling station back after tool return: ");
  Serial.print(toolName(tool));
  Serial.print(" ");
  Serial.print(nudgeDegrees);
  Serial.println(" deg.");

  moveStepper(nudgeDegrees);
  delay(toolStationSettleMs);
}

static void oscillateStationForToolReturn() {
  Serial.println("Oscillating station to settle returned tool.");

  for (int i = 0; i < toolReturnStationOscillationCycles; i++) {
    moveStepper(toolReturnStationOscillationDeg);
    moveStepper(-2.0 * toolReturnStationOscillationDeg);
    moveStepper(toolReturnStationOscillationDeg);
  }

  delay(toolStationSettleMs);
}

const char *toolName(ToolType tool) {
  if (tool == TOOL_GRIPPER) return "gripper";
  if (tool == TOOL_HAND) return "hand";
  if (tool == TOOL_DRILL) return "drill";
  return "none";
}

int toolSlot(ToolType tool) {
  if (tool == TOOL_GRIPPER) return toolSlotGripper;
  if (tool == TOOL_HAND) return toolSlotHand;
  if (tool == TOOL_DRILL) return toolSlotDrill;
  return -1;
}

bool setActiveTool(ToolType tool) {
  if (tool != TOOL_NONE && tool != TOOL_GRIPPER && tool != TOOL_HAND && tool != TOOL_DRILL) {
    Serial.println("Invalid tool.");
    return false;
  }

  activeTool = tool;

  if (activeTool == TOOL_NONE || activeTool == TOOL_HAND) {
    setToolPower(false);
  } else {
    setToolPower(true);
  }

  stepDelayMs = normalStepDelayMs;

  Serial.print("Active tool set to: ");
  Serial.println(toolName(activeTool));
  return true;
}

void setMotionSpeed(int speedMs) {
  normalStepDelayMs = constrain(speedMs, 5, 60);
  stepDelayMs = normalStepDelayMs;

  Serial.print("Speed updated. normalStepDelayMs = ");
  Serial.println(normalStepDelayMs);
  Serial.print("Current stepDelayMs = ");
  Serial.println(stepDelayMs);
  Serial.println("Smaller value = faster motion.");
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

static void moveToToolReturnOscillationPose() {
  Serial.println("Moving to partial return pose for station oscillation...");
  moveJointAnglesWithDelay(
    currentToolChangePitch,
    currentToolChangeRoll,
    currentToolChangeYaw,
    clampJoint(3, currentToolChangeElbow + toolReturnOscillationElbowOffset),
    toolDockSlowDelayMs
  );
  delay(toolStationSettleMs);
}

void moveIntoToolDockSlow() {
  Serial.println("Docking slowly from prechange to tool exchange pose with slower final elbow approach...");
  moveJointAnglesWithDelay(
    currentToolChangePitch,
    currentToolChangeRoll,
    currentToolChangeYaw,
    currentToolChangeElbow,
    toolDockFinalElbowDelayMs
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

static void prepareToolForReturn(ToolType tool) {
  Serial.print("Preparing tool for return: ");
  Serial.println(toolName(tool));

  if (tool == TOOL_GRIPPER) {
    gripperClose();
    delay(300);
  }

  setToolPower(false);
  delay(toolStationSettleMs);
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
  setMagnet(true);
  delay(toolMagnetSettleMs);
  shakeToolDockPose();
  delay(toolStationSettleMs);
  moveOutFromDockWithTool();
  setActiveTool(newTool);
  testPickedTool(newTool);
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

  prepareToolForReturn(oldTool);
  moveToToolPrechangePose();
  rotateToolStationToSlot(oldSlot);
  if (needsReturnStationNudge(oldTool)) {
    nudgeStationForToolReturn(oldTool, true);
  }
  moveToToolReturnOscillationPose();
  oscillateStationForToolReturn();
  moveIntoToolDockSlow();
  setToolPower(false);
  delay(toolStationSettleMs);
  setMagnet(false);
  delay(toolMagnetSettleMs);
  setActiveTool(TOOL_NONE);
  moveOutFromDockNoTool();
  if (needsReturnStationNudge(oldTool)) {
    nudgeStationForToolReturn(oldTool, false);
  }
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

  prepareToolForReturn(oldTool);
  moveToToolPrechangePose();
  rotateToolStationToSlot(oldSlot);
  if (needsReturnStationNudge(oldTool)) {
    nudgeStationForToolReturn(oldTool, true);
  }
  moveToToolReturnOscillationPose();
  oscillateStationForToolReturn();
  moveIntoToolDockSlow();
  setToolPower(false);
  delay(toolStationSettleMs);
  setMagnet(false);
  delay(toolMagnetSettleMs);
  activeTool = TOOL_NONE;
  moveOutFromDockNoTool();
  if (needsReturnStationNudge(oldTool)) {
    nudgeStationForToolReturn(oldTool, false);
  }

  rotateToolStationToSlot(newSlot);
  moveToToolPrechangePose();
  moveIntoToolDockSlow();
  setMagnet(true);
  delay(toolMagnetSettleMs);
  shakeToolDockPose();
  delay(toolStationSettleMs);
  moveOutFromDockWithTool();
  setActiveTool(newTool);
  testPickedTool(newTool);
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
  moveJointAngles(armRestPitch, armRestRoll, armRestYaw, armRestElbow);

  Serial.println("Rest pose done.");
}

void printCurrentServoAngles() {
  Serial.print("Raw servo angles: ");
  Serial.print("S0 = "); Serial.print(servo0Angle);
  Serial.print(", S1 = "); Serial.print(servo1Angle);
  Serial.print(", S2 = "); Serial.print(servo2Angle);
  Serial.print(", S3 = "); Serial.println(servo3Angle);
}

void testArmServos() {
  Serial.println("Arm servo diagnostic started.");
  Serial.println("Testing raw PWM on S0-S3 around rest pose.");

  moveAllSlow(servo0Rest, servo1Rest, servo2Rest, servo3Rest);
  delay(500);

  Serial.println("Testing S0 shoulder pitch...");
  moveServoSlow(servo0, servo0Angle, servo0Rest + 10, 0);
  delay(300);
  moveServoSlow(servo0, servo0Angle, servo0Rest - 10, 0);
  delay(300);
  moveServoSlow(servo0, servo0Angle, servo0Rest, 0);
  delay(300);

  Serial.println("Testing S1 shoulder roll...");
  moveServoSlow(servo1, servo1Angle, servo1Rest + 10, 1);
  delay(300);
  moveServoSlow(servo1, servo1Angle, servo1Rest - 10, 1);
  delay(300);
  moveServoSlow(servo1, servo1Angle, servo1Rest, 1);
  delay(300);

  Serial.println("Testing S2 shoulder yaw...");
  moveServoSlow(servo2, servo2Angle, servo2Rest + 10, 2);
  delay(300);
  moveServoSlow(servo2, servo2Angle, servo2Rest - 10, 2);
  delay(300);
  moveServoSlow(servo2, servo2Angle, servo2Rest, 2);
  delay(300);

  Serial.println("Testing S3 elbow pitch...");
  moveServoSlow(servo3, servo3Angle, servo3Rest + 10, 3);
  delay(300);
  moveServoSlow(servo3, servo3Angle, servo3Rest - 10, 3);
  delay(300);
  moveServoSlow(servo3, servo3Angle, servo3Rest, 3);

  currentPitch = 0;
  currentRoll = 0;
  currentYaw = 0;
  currentElbow = 0;

  Serial.println("Arm servo diagnostic done.");
  printCurrentServoAngles();
}

void setMagnet(bool enabled) {
  digitalWrite(magnetRelayPin, enabled ? HIGH : LOW);
  Serial.print("Magnet relay 1 is ");
  Serial.println(enabled ? "ON" : "OFF");
}

void setToolPower(bool enabled) {
  digitalWrite(toolPowerRelayPin, enabled ? HIGH : LOW);
  if (enabled && !toolPowerEnabled) {
    delay(toolPowerSettleMs);
  }
  toolPowerEnabled = enabled;

  Serial.print("Tool power relay 2 is ");
  Serial.println(enabled ? "ON" : "OFF");
}

void printToolStatus() {
  Serial.print("Active tool: ");
  Serial.println(toolName(activeTool));
  Serial.print("Tool station slot: ");
  Serial.println(currentToolStationSlot);
  Serial.println("Slots: 0=gripper, 1=drill, 2=hand");
}

void gripperOpen() {
  if (activeTool != TOOL_GRIPPER) {
    Serial.println("Warning: active tool is not gripper. Powering relay 2 and sending servo signal anyway.");
  }
  setToolPower(true);
  toolServo.write(toolServoRestAngle);
  delay(gripperServoMoveMs);
  Serial.println("Gripper open/rest command sent.");
}

void gripperClose() {
  if (activeTool != TOOL_GRIPPER) {
    Serial.println("Warning: active tool is not gripper. Powering relay 2 and sending servo signal anyway.");
  }
  setToolPower(true);
  toolServo.write(toolServoRunAngle);
  delay(gripperServoMoveMs);
  Serial.println("Gripper close/run command sent.");
}

void testPickedTool(ToolType tool) {
  Serial.print("Testing picked tool: ");
  Serial.println(toolName(tool));

  if (tool == TOOL_GRIPPER) {
    gripperOpen();
    gripperClose();
    gripperOpen();
  } else if (tool == TOOL_HAND) {
    Serial.println("Static hand model attached. No powered tool test needed.");
  } else if (tool == TOOL_DRILL) {
    setToolPower(true);
    delay(toolPickupTestRunMs);
    setToolPower(false);
  }

  Serial.println("Picked tool test done.");
}

static void runShowcaseTool(ToolType tool) {
  if (!pickupTool(tool)) {
    return;
  }

  Serial.print("Showcasing tool: ");
  Serial.println(toolName(tool));

  if (tool == TOOL_GRIPPER) {
    gripperClose();
    gripperOpen();
  } else if (tool == TOOL_HAND) {
    moveJointAngles(0, 100, 90, 100);
    delay(500);
    moveJointAngles(0, 60, 70, 100);
    delay(500);
    moveToRestPose();
  } else if (tool == TOOL_DRILL) {
    setToolPower(true);
    delay(toolShowcaseRunMs);
    setToolPower(false);
  }

  removeHeldTool();
}

void showcaseTools() {
  Serial.println("Tool showcase sequence started.");

  if (activeTool != TOOL_NONE) {
    Serial.println("Returning currently held tool before showcase.");
    removeHeldTool();
  }

  runShowcaseTool(TOOL_GRIPPER);
  runShowcaseTool(TOOL_HAND);
  runShowcaseTool(TOOL_DRILL);

  if (activeTool != TOOL_NONE) {
    removeHeldTool();
  }

  moveToRestPose();
  Serial.println("Tool showcase sequence done.");
}

void toolMotion() {
  Serial.println("Tool motion started.");

  if (activeTool == TOOL_DRILL) {
    setToolPower(true);
    Serial.print(toolName(activeTool));
    Serial.println(" is running. Use 'toolpower off' to stop it.");
  } else if (activeTool == TOOL_HAND) {
    Serial.println("Running static hand hello gesture.");
    moveJointAngles(0, 100, 90, 100);
    delay(500);
    moveJointAngles(0, 60, 70, 100);
    delay(500);
    moveToRestPose();
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
