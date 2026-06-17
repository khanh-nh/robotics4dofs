#include <Arduino.h>
#include "config.h"
#include "kinematics.h"
#include "robot_control.h"
#include "motions.h"

void printHelp();
bool parseFourValues(String command, int startIndex, float values[4]);
bool parseThreeValues(String command, int startIndex, float values[3]);
bool parseToolName(String toolText, ToolType &tool);
bool parseGeometryCommand(String command, String &pointName, float values[3]);

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

bool parseToolName(String toolText, ToolType &tool) {
  toolText.trim();

  if (toolText == "gripper" || toolText == "1") {
    tool = TOOL_GRIPPER;
    return true;
  }

  if (toolText == "hand" || toolText == "static" || toolText == "model" || toolText == "2") {
    tool = TOOL_HAND;
    return true;
  }

  if (toolText == "drill" || toolText == "dc" || toolText == "motor" || toolText == "3") {
    tool = TOOL_DRILL;
    return true;
  }

  if (toolText == "none" || toolText == "0") {
    tool = TOOL_NONE;
    return true;
  }

  return false;
}

bool parseGeometryCommand(String command, String &pointName, float values[3]) {
  int firstSpace = command.indexOf(' ');
  if (firstSpace < 0) return false;

  int secondSpace = command.indexOf(' ', firstSpace + 1);
  if (secondSpace < 0) return false;

  pointName = command.substring(firstSpace + 1, secondSpace);
  pointName.trim();

  return parseThreeValues(command, secondSpace + 1, values);
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
  Serial.println("P34 = [0, 25, -140]");
  Serial.println();

  Serial.println("Expected FK at rest:");
  Serial.println("fk 0 0 0 0 -> approximately x=0, y=160, z=30 mm");
  Serial.println();

  Serial.println("Commands:");
  Serial.println("rest");
  Serial.println("servotest                 Test raw PWM on arm servos S0-S3");
  Serial.println("q pitch roll yaw elbow    Example: q 90 80 70 60");
  Serial.println("fk pitch roll yaw elbow   Example: fk 90 80 70 60");
  Serial.println("go x y z                  Example: go 120 160 250");
  Serial.println("geom                      Print runtime geometry config");
  Serial.println("geom p01 x y z            Example: geom p34 0 25 -140");
  Serial.println("changepose                Print tool exchange pose");
  Serial.println("changepose p r y e        Example: changepose 0 -10 0 20");
  Serial.println("goto changepose           Move arm to tool exchange pose");
  Serial.println("tool                      Run active tool");
  Serial.println("tool gripper|hand|drill|none");
  Serial.println("pickup gripper|hand|drill");
  Serial.println("change gripper|hand|drill      Uses remembered held tool");
  Serial.println("change old new                 Example: change gripper hand");
  Serial.println("remove tool                    Return held tool to station");
  Serial.println("showcase                       Pick, run, return each tool, then rest");
  Serial.println("gripper open|close");
  Serial.println("magnet on|off             Relay 1: electric magnet tool lock");
  Serial.println("toolpower on|off          Relay 2: pogo VCC. Off stops drill/gripper.");
  Serial.println("station 0|1|2             Rotate exchange station to tool slot");
  Serial.println("station gripper|hand|drill");
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
  setupStepper();
  setupRelays();

  printHelp();

  Serial.println("Robot arm ready.");
  Serial.println("Open Serial Monitor at 115200 baud with Newline enabled.");
  printFK(armRestPitch, armRestRoll, armRestYaw, armRestElbow);
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
      printToolStatus();
      return;
    }

    if (command == "geom") {
      printGeometryConfig();
      return;
    }

    if (command == "changepose") {
      printToolChangePose();
      return;
    }

    if (command == "goto changepose") {
      moveToToolChangePose();
      return;
    }

    if (command.startsWith("changepose ")) {
      float values[4];

      if (parseFourValues(command, 11, values)) {
        setToolChangePose(values[0], values[1], values[2], values[3]);
      } else {
        Serial.println("Invalid changepose command. Use: changepose pitch roll yaw elbow");
      }
      return;
    }

    if (command.startsWith("geom ")) {
      String pointName;
      float values[3];

      if (parseGeometryCommand(command, pointName, values)) {
        char pointNameBuffer[12];
        pointName.toCharArray(pointNameBuffer, sizeof(pointNameBuffer));
        setGeometryPoint(pointNameBuffer, values[0], values[1], values[2]);
      } else {
        Serial.println("Invalid geom command. Use: geom base|p01|p12|p23|p34 x y z");
      }
      return;
    }

    if (command == "rest") {
      moveToRestPose();
      return;
    }

    if (command == "servotest") {
      testArmServos();
      return;
    }

    if (command == "tool") {
      toolMotion();
      return;
    }

    if (command == "relay on" || command == "magnet on") {
      setMagnet(true);
      return;
    }

    if (command == "relay off" || command == "magnet off") {
      setMagnet(false);
      return;
    }

    if (command == "toolpower on") {
      setToolPower(true);
      return;
    }

    if (command == "toolpower off") {
      setToolPower(false);
      return;
    }

    if (command == "gripper open") {
      gripperOpen();
      return;
    }

    if (command == "gripper close") {
      gripperClose();
      return;
    }

    if (command.startsWith("tool ")) {
      String toolText = command.substring(5);
      ToolType tool;

      if (parseToolName(toolText, tool)) {
        setActiveTool(tool);
      } else {
        Serial.println("Invalid tool command. Use: tool gripper|hand|drill|none");
      }
      return;
    }

    if (command.startsWith("pickup ")) {
      String toolText = command.substring(7);
      ToolType tool;

      if (parseToolName(toolText, tool) && tool != TOOL_NONE) {
        pickupTool(tool);
      } else {
        Serial.println("Invalid pickup command. Use: pickup gripper|hand|drill");
      }
      return;
    }

    if (command == "remove tool" || command == "remove" || command == "drop tool") {
      removeHeldTool();
      return;
    }

    if (command.startsWith("change ")) {
      String args = command.substring(7);
      args.trim();

      int spaceIndex = args.indexOf(' ');
      if (spaceIndex < 0) {
        ToolType newTool;
        if (parseToolName(args, newTool) && newTool != TOOL_NONE) {
          changeHeldTool(newTool);
        } else {
          Serial.println("Invalid change command. Use: change gripper|hand|drill");
        }
      } else {
        String oldText = args.substring(0, spaceIndex);
        String newText = args.substring(spaceIndex + 1);
        ToolType oldTool;
        ToolType newTool;

        if (parseToolName(oldText, oldTool) && parseToolName(newText, newTool) && newTool != TOOL_NONE) {
          if (activeTool != oldTool) {
            Serial.print("Remembered held tool was ");
            Serial.print(toolName(activeTool));
            Serial.print(", overriding to ");
            Serial.println(toolName(oldTool));
            activeTool = oldTool;
          }
          changeHeldTool(newTool);
        } else {
          Serial.println("Invalid change command. Use: change old new");
        }
      }
      return;
    }

    if (command.startsWith("station ")) {
      String stationText = command.substring(8);
      stationText.trim();

      if (stationText == "gripper") {
        rotateToolStationToSlot(toolSlotGripper);
      } else if (stationText == "hand") {
        rotateToolStationToSlot(toolSlotHand);
      } else if (stationText == "drill") {
        rotateToolStationToSlot(toolSlotDrill);
      } else {
        int slot = stationText.toInt();
        if (slot >= 0 && slot < toolStationSlots) {
          rotateToolStationToSlot(slot);
        } else {
          Serial.println("Invalid station command. Use: station 0|1|2 or station gripper|hand|drill");
        }
      }
      return;
    }

    if (command.startsWith("speed ")) {
      int newSpeed = command.substring(6).toInt();
      setMotionSpeed(newSpeed);
      return;
    }

    if (command.startsWith("stepper ")) {
      float degrees = command.substring(8).toFloat();
      moveStepper(degrees);
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

    if (command == "showcase") {
      showcaseTools();
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
