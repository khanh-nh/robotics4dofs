#include <Arduino.h>
#include "config.h"
#include "kinematics.h"
#include "robot_control.h"
#include "motions.h"

void printHelp();
bool parseFourValues(String command, int startIndex, float values[4]);
bool parseThreeValues(String command, int startIndex, float values[3]);

// Stepper Motor Pins
const int stepPin = 14;
const int dirPin = 12;

// =====================================================
// Parse helpers
// =====================================================
bool parseFourValues(String command, int startIndex, float values[4]) {
  int valueIndex = 0;

  while (valueIndex < 4 && startIndex < command.length()) {
    int nextSpace = command.indexOf(' ', startIndex);
    String valueText;

    if (nextSpace < 0) {
      valueText = command.substring(startIndex);
      startIndex = command.length();
    } else {
      valueText = command.substring(startIndex, nextSpace);
      startIndex = nextSpace + 1;
    }

    valueText.trim();

    if (valueText.length() > 0) {
      values[valueIndex] = valueText.toFloat();
      valueIndex++;
    }
  }

  return valueIndex == 4;
}

bool parseThreeValues(String command, int startIndex, float values[3]) {
  int valueIndex = 0;

  while (valueIndex < 3 && startIndex < command.length()) {
    int nextSpace = command.indexOf(' ', startIndex);
    String valueText;

    if (nextSpace < 0) {
      valueText = command.substring(startIndex);
      startIndex = command.length();
    } else {
      valueText = command.substring(startIndex, nextSpace);
      startIndex = nextSpace + 1;
    }

    valueText.trim();

    if (valueText.length() > 0) {
      values[valueIndex] = valueText.toFloat();
      valueIndex++;
    }
  }

  return valueIndex == 3;
}

// =====================================================
// Help menu
// =====================================================
void printHelp() {
  Serial.println();
  Serial.println("===== LEFT HUMANOID ARM CONTROL WITH FK + IK + TOOL =====");
  Serial.println();

  Serial.println("Servo mapping:");
  Serial.println("S0 = shoulder pitch = GPIO25");
  Serial.println("S1 = shoulder roll  = GPIO26");
  Serial.println("S2 = shoulder yaw   = GPIO32");
  Serial.println("S3 = elbow pitch    = GPIO33");
  Serial.println("Tool servo = GPIO27");
  Serial.println();

  Serial.println("Body/global coordinate frame:");
  Serial.println("Origin = center of hanging base/body");
  Serial.println("+x = forward");
  Serial.println("+y = left/outward");
  Serial.println("+z = upward");
  Serial.println();

  Serial.println("FK measurements:");
  Serial.println("basePoint = [0, 70, 515]");
  Serial.println("P01 = [0, 36, -14]");
  Serial.println("P12 = [0, 0, -85]");
  Serial.println("P23 = [0, 29, -205]");
  Serial.println("P34 = [0, 25, -181]");
  Serial.println();

  Serial.println("Expected FK at rest:");
  Serial.println("fk 0 0 0 0 -> approximately x=0, y=160, z=30 mm");
  Serial.println();

  Serial.println("Commands:");
  Serial.println("rest");
  Serial.println("q pitch roll yaw elbow    Example: q 90 80 70 60");
  Serial.println("fk pitch roll yaw elbow   Example: fk 90 80 70 60");
  Serial.println("go x y z                  Example: go 120 160 250");
  Serial.println("tool                      Run EE/tool servo briefly");
  Serial.println("speed value               Example: speed 10");
  Serial.println("all s0 s1 s2 s3           Example: all 135 20 150 125");
  Serial.println();

  Serial.println("Preset poses/motions:");
  Serial.println("raise");
  Serial.println("side");
  Serial.println("bend");
  Serial.println("wave");
  Serial.println("hello");
  Serial.println("stroke");
  Serial.println("handshake");
  Serial.println("pickplace");
  Serial.println("frontpick   Try to pick a low object on a table in front");
  Serial.println();

  Serial.println("status");
  Serial.println("help");
  Serial.println("========================================================");
  Serial.println();
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  setupServos();

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(relayPin, OUTPUT);

  printHelp();

  Serial.println("Robot arm ready.");
  Serial.println("Open Serial Monitor at 115200 baud with Newline enabled.");
  printFK(0, 0, 0, 0);
}

// =====================================================
// Main loop
// =====================================================
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) return;

    if (command == "help") {
      printHelp();
      return;
    }

    if (command == "status") {
      printCurrentServoAngles();
      Serial.print("Current q = [");
      Serial.print(currentPitch, 1);
      Serial.print(", ");
      Serial.print(currentRoll, 1);
      Serial.print(", ");
      Serial.print(currentYaw, 1);
      Serial.print(", ");
      Serial.print(currentElbow, 1);
      Serial.println("] deg");
      printFK(currentPitch, currentRoll, currentYaw, currentElbow);
      return;
    }

    if (command == "rest") {
      moveToRestPose();
      return;
    }

    if (command == "tool") {
      toolMotion();
      return;
    }

    if (command == "relay on") {
      digitalWrite(relayPin, HIGH);
      Serial.println("Relay is ON");
      return;
    }

    if (command == "relay off") {
      digitalWrite(relayPin, LOW);
      Serial.println("Relay is OFF");
      return;
    }

    if (command.startsWith("speed ")) {
      int newSpeed = command.substring(6).toInt();
      stepDelayMs = constrain(newSpeed, 5, 60);
      Serial.print("Speed updated. stepDelayMs = ");
      Serial.println(stepDelayMs);
      Serial.println("Smaller value = faster motion.");
      return;
    }

    if (command.startsWith("stepper ")) {
      float degrees = command.substring(8).toFloat();
      
      // 3200 steps for 1 full revolution (1/16 microstepping)
      int steps = abs(degrees) * (3200.0 / 360.0) ;
      
      if (degrees >= 0) {
        digitalWrite(dirPin, HIGH); // Set direction forward
      } else {
        digitalWrite(dirPin, LOW);  // Set direction backward
      }
      
      // Create pulses for steps
      for (int x = 0; x < steps; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
      }
      return;
    }

    if (command.startsWith("fk ")) {
      float values[4];
      if (parseFourValues(command, 3, values)) {
        float pitch = clampJoint(0, values[0]);
        float roll  = clampJoint(1, values[1]);
        float yaw   = clampJoint(2, values[2]);
        float elbow = clampJoint(3, values[3]);
        printFK(pitch, roll, yaw, elbow);
      } else {
        Serial.println("Invalid fk command. Use: fk pitch roll yaw elbow");
      }
      return;
    }

    if (command.startsWith("go ")) {
      float values[3];
      if (parseThreeValues(command, 3, values)) {
        Vec3 target;
        target.x = values[0];
        target.y = values[1];
        target.z = values[2];
        moveToXYZ(target);
      } else {
        Serial.println("Invalid go command. Use: go x y z");
      }
      return;
    }

    if (command == "raise") {
      moveJointAngles(120, 0, 0, 0);
      return;
    }

    if (command == "side") {
      moveJointAngles(0, 60, 0, 0);
      return;
    }

    if (command == "bend") {
      moveJointAngles(0, 0, 0, 70);
      return;
    }

    if (command == "wave") {
      waveMotion();
      return;
    }

    if (command == "hello") {
      helloMotion();
      return;
    }

    if (command == "stroke") {
      strokingMotion();
      return;
    }

    if (command == "handshake") {
      handShakeMotion();
      return;
    }

    if (command == "pickplace") {
      pickAndPlaceMotion();
      return;
    }

    if (command == "frontpick") {
      frontPickMotion();
      return;
    }

    if (command.startsWith("q ")) {
      float values[4];
      if (parseFourValues(command, 2, values)) {
        moveJointAngles(values[0], values[1], values[2], values[3]);
      } else {
        Serial.println("Invalid q command. Use: q pitch roll yaw elbow");
      }
      return;
    }

    if (command.startsWith("all ")) {
      float values[4];
      if (parseFourValues(command, 4, values)) {
        moveAllSlow(round(values[0]), round(values[1]), round(values[2]), round(values[3]));
        Serial.println("Warning: raw servo control does not update current q state.");
        Serial.println("Use q or go before IK for best behavior.");
      } else {
        Serial.println("Invalid all command. Use: all s0 s1 s2 s3");
      }
      return;
    }

    Serial.println("Invalid command. Type help for command list.");
  }
}
