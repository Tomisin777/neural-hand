/*
  MirrorHand - Standalone Single Servo Diagnostic (Arduino UNO R4 WiFi)
  =====================================================================
  Sweeps one servo on Pin 9 slowly between 20 and 140 degrees while printing
  the current angle to the Serial Monitor at 115200 baud.

  Use this to:
  1. Verify servo electrical connections and separate 5V supply before mounting on glove.
  2. Confirm smooth motion and verify the servo does not hit mechanical hard stops.
*/

#include <Servo.h>

// Configuration Constants
const int SERVO_PIN = 9;              // Pin under test (Index servo pin)
const int TEST_MIN_ANGLE = 20;        // Minimum safe test angle (degrees)
const int TEST_MAX_ANGLE = 140;       // Maximum safe test angle (degrees)
const int STEP_DELAY_MS = 25;         // Delay between 1-degree steps (controls sweep speed)
const int PAUSE_AT_LIMITS_MS = 1000;  // Pause at endpoints (ms)

Servo testServo;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Short wait for serial port initialization

  Serial.println(F("=============================================="));
  Serial.println(F("  MIRRORHAND ARDUINO SINGLE SERVO BENCH TEST  "));
  Serial.print(F("  Testing Pin: "));
  Serial.print(SERVO_PIN);
  Serial.print(F(" | Range: "));
  Serial.print(TEST_MIN_ANGLE);
  Serial.print(F(" deg -> "));
  Serial.print(TEST_MAX_ANGLE);
  Serial.println(F(" deg"));
  Serial.println(F("=============================================="));

  testServo.attach(SERVO_PIN);
  testServo.write(TEST_MIN_ANGLE);
  delay(1000);
}

void loop() {
  static int sweepCount = 0;
  sweepCount++;

  Serial.println();
  Serial.print(F("--- Starting Sweep #"));
  Serial.print(sweepCount);
  Serial.print(F(" (Forward: "));
  Serial.print(TEST_MIN_ANGLE);
  Serial.print(F(" -> "));
  Serial.print(TEST_MAX_ANGLE);
  Serial.println(F(") ---"));

  // Forward sweep: MIN to MAX
  for (int angle = TEST_MIN_ANGLE; angle <= TEST_MAX_ANGLE; angle++) {
    testServo.write(angle);
    if (angle % 20 == 0) {
      Serial.print(F("  Angle: "));
      Serial.print(angle);
      Serial.println(F(" deg"));
    }
    delay(STEP_DELAY_MS);
  }

  Serial.print(F("Reached MAX limit ("));
  Serial.print(TEST_MAX_ANGLE);
  Serial.println(F(" deg). Pausing..."));
  delay(PAUSE_AT_LIMITS_MS);

  Serial.print(F("--- Starting Sweep #"));
  Serial.print(sweepCount);
  Serial.print(F(" (Reverse: "));
  Serial.print(TEST_MAX_ANGLE);
  Serial.print(F(" -> "));
  Serial.print(TEST_MIN_ANGLE);
  Serial.println(F(") ---"));

  // Reverse sweep: MAX to MIN
  for (int angle = TEST_MAX_ANGLE; angle >= TEST_MIN_ANGLE; angle--) {
    testServo.write(angle);
    if (angle % 20 == 0) {
      Serial.print(F("  Angle: "));
      Serial.print(angle);
      Serial.println(F(" deg"));
    }
    delay(STEP_DELAY_MS);
  }

  Serial.print(F("Reached MIN limit ("));
  Serial.print(TEST_MIN_ANGLE);
  Serial.println(F(" deg). Pausing..."));
  delay(PAUSE_AT_LIMITS_MS);
}
