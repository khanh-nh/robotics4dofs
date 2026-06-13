#include "kinematics.h"
#include "config.h"
#include "robot_control.h"
#include <math.h>
#include <Arduino.h>

float degToRad(float deg) {
  return deg * PI / 180.0;
}

Vec3 makeVec(const float p[3]) {
  Vec3 v;
  v.x = p[0];
  v.y = p[1];
  v.z = p[2];
  return v;
}

Vec3 vecAdd(Vec3 a, Vec3 b) {
  Vec3 result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

Mat3 identityMat() {
  Mat3 I;
  I.m[0][0] = 1; I.m[0][1] = 0; I.m[0][2] = 0;
  I.m[1][0] = 0; I.m[1][1] = 1; I.m[1][2] = 0;
  I.m[2][0] = 0; I.m[2][1] = 0; I.m[2][2] = 1;
  return I;
}

Mat3 matMul(Mat3 A, Mat3 B) {
  Mat3 C;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C.m[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        C.m[i][j] += A.m[i][k] * B.m[k][j];
      }
    }
  }
  return C;
}

Vec3 matVecMul(Mat3 A, Vec3 v) {
  Vec3 result;
  result.x = A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z;
  result.y = A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z;
  result.z = A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z;
  return result;
}

Mat3 rotX(float deg) {
  float r = degToRad(deg);
  float c = cos(r);
  float s = sin(r);

  Mat3 R;
  R.m[0][0] = 1; R.m[0][1] = 0;  R.m[0][2] = 0;
  R.m[1][0] = 0; R.m[1][1] = c;  R.m[1][2] = -s;
  R.m[2][0] = 0; R.m[2][1] = s;  R.m[2][2] = c;
  return R;
}

Mat3 rotY(float deg) {
  float r = degToRad(deg);
  float c = cos(r);
  float s = sin(r);

  Mat3 R;
  R.m[0][0] = c;  R.m[0][1] = 0; R.m[0][2] = s;
  R.m[1][0] = 0;  R.m[1][1] = 1; R.m[1][2] = 0;
  R.m[2][0] = -s; R.m[2][1] = 0; R.m[2][2] = c;
  return R;
}

Mat3 rotZ(float deg) {
  float r = degToRad(deg);
  float c = cos(r);
  float s = sin(r);

  Mat3 R;
  R.m[0][0] = c; R.m[0][1] = -s; R.m[0][2] = 0;
  R.m[1][0] = s; R.m[1][1] = c;  R.m[1][2] = 0;
  R.m[2][0] = 0; R.m[2][1] = 0;  R.m[2][2] = 1;
  return R;
}

Vec3 calculateFK(float pitchDeg, float rollDeg, float yawDeg, float elbowDeg) {
  Vec3 p;
  p.x = basePoint[0];
  p.y = basePoint[1];
  p.z = basePoint[2];

  Mat3 R = identityMat();

  R = matMul(R, rotY(-pitchDeg));
  p = vecAdd(p, matVecMul(R, makeVec(P01)));

  R = matMul(R, rotX(rollDeg));
  p = vecAdd(p, matVecMul(R, makeVec(P12)));

  R = matMul(R, rotZ(yawDeg));
  p = vecAdd(p, matVecMul(R, makeVec(P23)));

  R = matMul(R, rotY(-elbowDeg));
  p = vecAdd(p, matVecMul(R, makeVec(P34)));

  return p;
}

void printFK(float pitch, float roll, float yaw, float elbow) {
  Vec3 ee = calculateFK(pitch, roll, yaw, elbow);

  Serial.println();
  Serial.print("FK input q = [");
  Serial.print(pitch, 1);
  Serial.print(", ");
  Serial.print(roll, 1);
  Serial.print(", ");
  Serial.print(yaw, 1);
  Serial.print(", ");
  Serial.print(elbow, 1);
  Serial.println("] deg");

  Serial.println("EE position in BODY/GLOBAL frame [mm]:");
  Serial.print("x = "); Serial.println(ee.x, 2);
  Serial.print("y = "); Serial.println(ee.y, 2);
  Serial.print("z = "); Serial.println(ee.z, 2);
  Serial.println();
}

float positionErrorSq(Vec3 target, float q[4]) {
  Vec3 p = calculateFK(q[0], q[1], q[2], q[3]);
  float dx = p.x - target.x;
  float dy = p.y - target.y;
  float dz = p.z - target.z;
  return dx * dx + dy * dy + dz * dz;
}

float solveIKFromSeed(Vec3 target, float seed[4], float solution[4]) {
  float q[4];
  for (int i = 0; i < 4; i++) {
    q[i] = clampJoint(i, seed[i]);
  }

  float bestErrSq = positionErrorSq(target, q);
  float stepSizes[] = {30, 15, 8, 4, 2, 1, 0.5};
  int numSteps = sizeof(stepSizes) / sizeof(stepSizes[0]);

  for (int s = 0; s < numSteps; s++) {
    float step = stepSizes[s];
    bool improved = true;
    int iter = 0;

    while (improved && iter < 25) {
      improved = false;
      iter++;

      for (int joint = 0; joint < 4; joint++) {
        float original = q[joint];

        q[joint] = clampJoint(joint, original + step);
        float errPlus = positionErrorSq(target, q);

        q[joint] = clampJoint(joint, original - step);
        float errMinus = positionErrorSq(target, q);

        if (errPlus < bestErrSq && errPlus <= errMinus) {
          q[joint] = clampJoint(joint, original + step);
          bestErrSq = errPlus;
          improved = true;
        } else if (errMinus < bestErrSq) {
          q[joint] = clampJoint(joint, original - step);
          bestErrSq = errMinus;
          improved = true;
        } else {
          q[joint] = original;
        }
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    solution[i] = q[i];
  }

  return bestErrSq;
}

bool solveIK(Vec3 target, float solution[4], float &finalError) {
  float bestQ[4];
  float tempQ[4];
  float bestErrSq = 999999999.0;

  float seeds[6][4] = {
    {currentPitch, currentRoll, currentYaw, currentElbow},
    {0, 0, 0, 0},
    {30, 30, 0, 60},
    {60, 60, 0, 80},
    {90, 80, 40, 60},
    {90, 80, 100, 80}
  };

  for (int i = 0; i < 6; i++) {
    float errSq = solveIKFromSeed(target, seeds[i], tempQ);
    if (errSq < bestErrSq) {
      bestErrSq = errSq;
      for (int j = 0; j < 4; j++) {
        bestQ[j] = tempQ[j];
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    solution[i] = bestQ[i];
  }

  finalError = sqrt(bestErrSq);
  return finalError <= IK_ACCEPT_ERROR_MM;
}