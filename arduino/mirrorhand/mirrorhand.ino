/*
  MirrorHand - Arduino UNO R4 WiFi Firmware
  ==========================================
  Controls 1 or 2 Tower Pro SG92R micro servos on an assistive rehabilitation glove.
  Receives normalized finger curl setpoints from laptop over USB serial at 115200 baud:
      "T:<thumb_curl>,I:<index_curl>\n" (e.g., "T:0.42,I:0.80\n")

  Hardware Configuration:
  - Microcontroller: Arduino UNO R4 WiFi (Renesas RA4M1)
  - Thumb Servo (Signal): Pin 10 (PWM)
  - Index Servo (Signal): Pin 9 (PWM)
  - Kill Switch Button:   Pin 2 (INPUT_PULLUP, pressed = LOW to GND)
  - Status LED:          LED_BUILTIN (Pin 13)
  - Power:               External 5V supply with common ground to Arduino GND

  Failsafes:
  - Velocity-limited angle ramping (MAX_STEP_PER_TICK) to prevent 5V current spikes.
  - Watchdog timeout: relaxes servos to 0% (open) if no serial data for 1.0 second.
  - Hardware kill switch on Pin 2: forces instant relaxation while pressed.
*/

#include <Servo.h>

// ==============================================================================
// CONFIGURATION & TUNABLE CONSTANTS
// ==============================================================================

// Hardware Channel Setup
// Set to 1 if driving only the Index servo on Pin 9; set to 2 for Thumb (Pin 10) + Index (Pin 9).
const int NUM_SERVOS = 2;

// Pin Assignments
const int PIN_SERVO_THUMB = 10;      // Thumb tendon servo signal
const int PIN_SERVO_INDEX = 9;       // Index finger tendon servo signal
const int PIN_KILL_SWITCH = 2;       // Pushbutton to GND (uses internal pull-up)
const int STATUS_LED = 13;     // Built-in LED on Pin 13

// Serial Communication
const unsigned long SERIAL_BAUD = 115200;

// Safety Mechanical Angle Limits (Degrees: 0 to 180)
// Calibrate these limits so the servo pulls the tendon without stalling or straining fingers!
const float INDEX_MIN_ANGLE = 20.0f;  // Fully OPEN position angle
const float INDEX_MAX_ANGLE = 140.0f; // Fully CLOSED position angle
const bool  INDEX_INVERT = false;     // True if 1.0 curl requires rotating toward MIN_ANGLE

const float THUMB_MIN_ANGLE = 20.0f;  // Fully OPEN position angle
const float THUMB_MAX_ANGLE = 140.0f; // Fully CLOSED position angle
const bool  THUMB_INVERT = false;     // True if 1.0 curl requires rotating toward MIN_ANGLE

// Motion Ramping & Rate Limiting
// Max degrees per tick (prevents current spikes on shared 5V supply)
const float MAX_STEP_PER_TICK = 60.0f; // ~75 deg/sec at 50Hz tick rate
const unsigned long LOOP_TICK_MS = 20; // 20ms tick = 50 Hz control loop

// Safety Watchdog Timeout
const unsigned long SERIAL_TIMEOUT_MS = 1000; // Relax servos if no data for 1000ms

// ==============================================================================
// RAMPED SERVO CONTROLLER CLASS
// ==============================================================================

class RampedServo {
private:
  Servo _servo;
  int _pin;
  float _minAngle;
  float _maxAngle;
  bool _invert;
  float _currentAngle;
  float _targetAngle;

  float curlToAngle(float curl) {
    curl = constrain(curl, 0.0f, 1.0f);
    if (_invert) {
      return _maxAngle - curl * (_maxAngle - _minAngle);
    } else {
      return _minAngle + curl * (_maxAngle - _minAngle);
    }
  }

public:
  RampedServo(int pin, float minAng, float maxAng, bool inv)
    : _pin(pin), _minAngle(minAng), _maxAngle(maxAng), _invert(inv),
      _currentAngle(minAng), _targetAngle(minAng) {}

  void attach() {
    _currentAngle = curlToAngle(0.0f);
    _targetAngle = _currentAngle;
    _servo.attach(_pin);
    _servo.write((int)(_currentAngle + 0.5f));
  }

  void setTargetCurl(float curl) {
    _targetAngle = curlToAngle(curl);
  }

  void updateRamp(float maxStep) {
    float diff = _targetAngle - _currentAngle;
    if (abs(diff) <= maxStep) {
      _currentAngle = _targetAngle;
    } else if (diff > 0) {
      _currentAngle += maxStep;
    } else {
      _currentAngle -= maxStep;
    }
    _servo.write((int)(_currentAngle + 0.5f));
  }

  float getCurrentAngle() const {
    return _currentAngle;
  }
};

// Instantiate Servo Controllers
RampedServo servoIndex(PIN_SERVO_INDEX, INDEX_MIN_ANGLE, INDEX_MAX_ANGLE, INDEX_INVERT);
RampedServo servoThumb(PIN_SERVO_THUMB, THUMB_MIN_ANGLE, THUMB_MAX_ANGLE, THUMB_INVERT);

// ==============================================================================
// STATE VARIABLES & BUFFERS
// ==============================================================================

char rxBuffer[64];
uint8_t rxIndex = 0;
unsigned long lastPacketTime = 0;
unsigned long lastTickTime = 0;

// ==============================================================================
// PACKET PARSER
// ==============================================================================

bool parsePacket(const char* buffer, float &outThumb, float &outIndex) {
  // Format: "T:0.42,I:0.80" or "I:0.80,T:0.42" or single "I:0.80"
  bool hasIndex = false;
  bool hasThumb = false;
  float tVal = 0.0f;
  float iVal = 0.0f;

  const char* ptr = buffer;
  while (*ptr) {
    // Skip leading spaces or commas
    while (*ptr == ' ' || *ptr == ',' || *ptr == '\t') ptr++;
    if (!*ptr) break;

    char key = *ptr;
    if ((key == 'T' || key == 't' || key == 'I' || key == 'i') && *(ptr + 1) == ':') {
      ptr += 2; // Skip key and colon
      char* endPtr;
      float val = strtod(ptr, &endPtr);
      if (ptr != endPtr) {
        val = constrain(val, 0.0f, 1.0f);
        if (key == 'T' || key == 't') {
          tVal = val;
          hasThumb = true;
        } else {
          iVal = val;
          hasIndex = true;
        }
        ptr = endPtr;
      } else {
        ptr++;
      }
    } else {
      ptr++;
    }
  }

  if (hasIndex) {
    outIndex = iVal;
    outThumb = hasThumb ? tVal : iVal; // Mirror index if thumb is absent
    return true;
  }
  return false;
}

// ==============================================================================
// SETUP
// ==============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Initialize Kill Switch Pin (Internal Pull-Up)
  pinMode(PIN_KILL_SWITCH, INPUT_PULLUP);

  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Attach active servos
  servoIndex.attach();
  if (NUM_SERVOS >= 2) {
    servoThumb.attach();
  }

  lastPacketTime = millis();
  lastTickTime = millis();

  Serial.println(F("=============================================="));
  Serial.println(F("  MIRRORHAND ARDUINO UNO R4 FIRMWARE READY    "));
  Serial.print(F("  Config: NUM_SERVOS="));
  Serial.print(NUM_SERVOS);
  Serial.println(F(" | Baud: 115200"));
  Serial.println(F("=============================================="));
}

// ==============================================================================
// MAIN RUNTIME LOOP
// ==============================================================================

void loop() {
  unsigned long now = millis();

  // ----------------------------------------------------------------------------
  // 1. FAILSAFE CHECK: Hardware Kill Switch (Pin 2)
  // ----------------------------------------------------------------------------
  if (digitalRead(PIN_KILL_SWITCH) == LOW) {
    // Button connected Pin 2 to GND -> Force all servos to relax immediately
    servoIndex.setTargetCurl(0.0f);
    if (NUM_SERVOS >= 2) {
      servoThumb.setTargetCurl(0.0f);
    }

    // Step ramping quickly toward open
    servoIndex.updateRamp(MAX_STEP_PER_TICK * 2.0f);
    if (NUM_SERVOS >= 2) {
      servoThumb.updateRamp(MAX_STEP_PER_TICK * 2.0f);
    }

    // Rapid blink LED to indicate kill switch engaged
    digitalWrite(STATUS_LED, (now / 100) % 2);

    // Flush any pending serial bytes
    while (Serial.available() > 0) {
      Serial.read();
    }
    rxIndex = 0;

    delay(LOOP_TICK_MS);
    return;
  }

  // ----------------------------------------------------------------------------
  // 2. NON-BLOCKING SERIAL RECEPTION & PARSING
  // ----------------------------------------------------------------------------
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';
        float thumbCurl = 0.0f;
        float indexCurl = 0.0f;
        if (parsePacket(rxBuffer, thumbCurl, indexCurl)) {
          servoIndex.setTargetCurl(indexCurl);
          if (NUM_SERVOS >= 2) {
            servoThumb.setTargetCurl(indexCurl);
          }
          lastPacketTime = now;
        }
        rxIndex = 0;
      }
    } else {
      if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        rxIndex = 0; // Prevent buffer overflow
      }
    }
  }

  // ----------------------------------------------------------------------------
  // 3. FAILSAFE CHECK: Watchdog Timeout (1.0 Second)
  // ----------------------------------------------------------------------------
  if (now - lastPacketTime > SERIAL_TIMEOUT_MS) {
    // No valid packet recently -> safety ramp back to open
    servoIndex.setTargetCurl(0.0f);
    if (NUM_SERVOS >= 2) {
      servoThumb.setTargetCurl(0.0f);
    }
    // Slow blink LED to indicate waiting/timeout
    digitalWrite(STATUS_LED, (now / 500) % 2);
  } else {
    // Serial link healthy -> solid LED
    digitalWrite(STATUS_LED, HIGH);
  }

  // ----------------------------------------------------------------------------
  // 4. STEP MOTION RAMPING (Periodic at LOOP_TICK_MS)
  // ----------------------------------------------------------------------------
  if (now - lastTickTime >= LOOP_TICK_MS) {
    lastTickTime = now;
    servoIndex.updateRamp(MAX_STEP_PER_TICK);
    if (NUM_SERVOS >= 2) {
      servoThumb.updateRamp(MAX_STEP_PER_TICK);
    }
  }
}
