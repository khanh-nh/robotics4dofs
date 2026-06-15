#pragma once
#include <Arduino.h>

// =====================================================
// Pin mapping for normal ESP32
// =====================================================
const int servo0Pin = 25; // shoulder pitch
const int servo1Pin = 26; // shoulder roll
const int servo2Pin = 32; // shoulder yaw
const int servo3Pin = 33; // elbow pitch
const int toolServoPin = 27; // tool / gripper / EE servo

// =====================================================
// Stepper Motor Settings (A4988)
// =====================================================
const int stepperStepPin = 14;
const int stepperDirPin  = 12;
const int stepperStepsPerRev = 3200; // 1.8 degree step with 1/16 microstepping (200 * 16)


// Relay Settings
// =====================================================
const int magnetRelayPin = 21;    // Relay 1: electric magnet tool lock
const int toolPowerRelayPin = 19; // Relay 2: pogo VCC for gripper/pump/drill

// Backward-compatible alias for older code/commands.
const int relayPin = magnetRelayPin;

// =====================================================

// =====================================================
// Tool exchange station settings
// Three tools on a 360 degree rotary plate, 120 degrees apart.
// =====================================================
const int toolStationSlots = 3;
const float toolStationStepDeg = 120.0;

const int toolSlotGripper = 0;
const int toolSlotVacuum = 1;
const int toolSlotDrill = 2;

// Tool exchange arm pose, expressed as joint angles relative to rest.
// Start near rest; tune these once the physical station position is fixed.
const float toolChangePitch = 0;
const float toolChangeRoll = 0;
const float toolChangeYaw = 0;
const float toolChangeElbow = 0;

const int toolMagnetSettleMs = 500;
const int toolStationSettleMs = 500;

// =====================================================
// Servo PWM settings
// =====================================================
const int minPulse = 500;
const int maxPulse = 2400;

// =====================================================
// Tool servo settings
// Normal servo: 90 = rest, 150 = active
// Continuous servo: 90 = stop, 150/180 = rotate
// =====================================================
const int toolServoRestAngle = 180;
const int toolServoRunAngle  = 150;
const int toolRunTimeMs      = 700;

// =====================================================
// Rest pose raw servo angles
// =====================================================
const int servo0Rest = 130; // shoulder pitch 126
const int servo1Rest = 17;  // shoulder roll 15
const int servo2Rest = 150; // shoulder yaw
const int servo3Rest = 127; // elbow pitch

// =====================================================
// Direction for LEFT arm
// servo angle = rest + dir * joint angle
// =====================================================
const int servo0Dir = -1; // shoulder pitch
const int servo1Dir =  1; // shoulder roll
const int servo2Dir = -1; // shoulder yaw
const int servo3Dir = -1; // elbow pitch

// =====================================================
// Raw servo safety limits
// =====================================================
const int servo0Min = 0;
const int servo0Max = 180;

const int servo1Min = 0;  // shoulder roll limit
const int servo1Max = 180;

const int servo2Min = 0;
const int servo2Max = 180;

const int servo3Min = 0;
const int servo3Max = 180;

// =====================================================
// Joint command limits relative to rest pose
// =====================================================
const float pitchMin = -50;
const float pitchMax = 130;

const float rollMin = 0;
const float rollMax = 160;

const float yawMin = -30;
const float yawMax = 150;

const float elbowMin = -55;
const float elbowMax = 125;

// =====================================================
// IK settings
// =====================================================
const float IK_ACCEPT_ERROR_MM = 35.0;

// =====================================================
// Body/global coordinate frame
// Origin = center of hanging base/body
// +x = forward
// +y = left/outward
// +z = upward
// =====================================================
const float basePoint[3] = {0, 70, 515};

const float P01[3] = {0, 36, -14};
const float P12[3] = {0, 0, -85};
const float P23[3] = {0, 29, -205};
const float P34[3] = {0, 25, -140};
