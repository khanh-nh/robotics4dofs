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

  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      servo.write(angle);
      delay(stepDelayMs);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      servo.write(angle);
      delay(stepDelayMs);
    }
  }

  currentAngle = targetAngle;
}

void moveAllSlow(int target0, int target1, int target2, int target3) {
  target0 = clampServoAngle(0, target0);
  target1 = clampServoAngle(1, target1);
  target2 = clampServoAngle(2, target2);
  target3 = clampServoAngle(3, target3);

  while (
    servo0Angle != target0 ||
    servo1Angle != target1 ||
    servo2Angle != target2 ||
    servo3Angle != target3
  ) {
    if (servo0Angle < target0) servo0Angle++;
    else if (servo0Angle > target0) servo0Angle--;

    if (servo1Angle < target1) servo1Angle++;
    else if (servo1Angle > target1) servo1Angle--;

    if (servo2Angle < target2) servo2Angle++;
    else if (servo2Angle > target2) servo2Angle--;

    if (servo3Angle < target3) servo3Angle++;
    else if (servo3Angle > target3) servo3Angle--;

    servo0.write(servo0Angle);
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);

    delay(stepDelayMs);
  }

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

void toolMotion() {
  Serial.println("Tool motion started.");
  toolServo.write(toolServoRunAngle);
  delay(toolRunTimeMs);
  toolServo.write(toolServoRestAngle);
  delay(300);
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