#pragma once

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Mat3 {
  float m[3][3];
};

float degToRad(float deg);
Vec3 makeVec(const float p[3]);
Vec3 vecAdd(Vec3 a, Vec3 b);
Mat3 identityMat();
Mat3 matMul(Mat3 A, Mat3 B);
Vec3 matVecMul(Mat3 A, Vec3 v);
Mat3 rotX(float deg);
Mat3 rotY(float deg);
Mat3 rotZ(float deg);

Vec3 calculateFK(float pitchDeg, float rollDeg, float yawDeg, float elbowDeg);
void printFK(float pitch, float roll, float yaw, float elbow);
bool setGeometryPoint(const char *pointName, float x, float y, float z);
void printGeometryConfig();

float positionErrorSq(Vec3 target, float q[4]);
float solveIKFromSeed(Vec3 target, float seed[4], float solution[4]);
bool solveIK(Vec3 target, float solution[4], float &finalError);
