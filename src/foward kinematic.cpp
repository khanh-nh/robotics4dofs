#include <Arduino.h>
#include <math.h>

// Simple 4DOF humanoid arm forward kinematics helper.
// Name kept as requested: "foward kinematic.cpp".
//
// Joint inputs are relative to rest pose, same as the main "q" command:
//   pitch: shoulder forward/back
//   roll:  shoulder side raise
//   yaw:   shoulder twist
//   elbow: elbow bend
//
// Units:
//   angles: degrees
//   lengths: millimeters
//
// Adjust these two lengths to match your robot.
const float upperArmLengthMm = 100.0f;
const float forearmLengthMm = 100.0f;

struct ArmPose {
  float x;
  float y;
  float z;
  float reach;
};

static float fkDegToRad(float degrees) {
  return degrees * PI / 180.0f;
}

ArmPose computeForwardKinematics(float pitchDeg, float rollDeg, float yawDeg, float elbowDeg) {
  float pitch = fkDegToRad(pitchDeg);
  float roll = fkDegToRad(rollDeg);
  float yaw = fkDegToRad(yawDeg);
  float elbow = fkDegToRad(elbowDeg);

  float shoulderReach = upperArmLengthMm * cos(pitch);
  float elbowX = shoulderReach * cos(yaw);
  float elbowY = shoulderReach * sin(yaw);
  float elbowZ = upperArmLengthMm * sin(pitch);

  float forearmPitch = pitch + elbow;
  float forearmReach = forearmLengthMm * cos(forearmPitch);
  float wristX = elbowX + forearmReach * cos(yaw);
  float wristY = elbowY + forearmReach * sin(yaw);
  float wristZ = elbowZ + forearmLengthMm * sin(forearmPitch);

  wristY += (upperArmLengthMm + forearmLengthMm) * sin(roll);

  ArmPose pose;
  pose.x = wristX;
  pose.y = wristY;
  pose.z = wristZ;
  pose.reach = sqrt(wristX * wristX + wristY * wristY + wristZ * wristZ);
  return pose;
}

void printForwardKinematics(float pitchDeg, float rollDeg, float yawDeg, float elbowDeg) {
  ArmPose pose = computeForwardKinematics(pitchDeg, rollDeg, yawDeg, elbowDeg);

  Serial.print("FK pose mm: X=");
  Serial.print(pose.x);
  Serial.print(", Y=");
  Serial.print(pose.y);
  Serial.print(", Z=");
  Serial.print(pose.z);
  Serial.print(", reach=");
  Serial.println(pose.reach);
}
