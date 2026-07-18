/*
 * OIRS 4-Mirror Auto-Alignment -- ESP32 Servo Controller
 * ======================================================
 * Controls 8x MG90 servos (4 mirrors x 2 axes) via PCA9685 (I2C).
 *
 * Hardware:
 *   - ESP32 dev board
 *   - PCA9685 at I2C address 0x40
 *     - Channel 0: M0 Pan    Channel 1: M0 Tilt
 *     - Channel 2: M1 Pan    Channel 3: M1 Tilt
 *     - Channel 4: M2 Pan    Channel 5: M2 Tilt
 *     - Channel 6: M3 Pan    Channel 7: M3 Tilt
 *   - I2C: SDA = GPIO 21, SCL = GPIO 22
 *
 * Serial protocol (115200 baud):
 *   Receive: ALL:p0,t0,p1,t1,p2,t2,p3,t3\n
 *   Send:    OK:ALL:p0,t0,p1,t1,p2,t2,p3,t3\n
 *   Receive: PING\n
 *   Send:    PONG\n
 *
 * Required library:
 *   Adafruit PWM Servo Driver Library
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── Configuration ────────────────────────────────────────────────────
#define PCA9685_ADDR     0x48
#define I2C_SDA          21
#define I2C_SCL          22

#define NUM_MIRRORS      4
#define NUM_SERVOS       8    // 4 mirrors x 2 axes

#define SERIAL_BAUD      115200

// MG90 servo pulse width range (PCA9685 ticks at 50 Hz)
// 50 Hz -> 20 ms period -> 4096 ticks / 20 ms -> 1 tick ~ 4.88 us
// MG90 typical: 500 us (0 deg) to 2400 us (180 deg)
#define SERVO_MIN_TICKS  102   // ~500 us  -> 0 deg
#define SERVO_MAX_TICKS  491   // ~2400 us -> 180 deg

// Safe angle limits
#define SAFE_MIN_ANGLE   30.0
#define SAFE_MAX_ANGLE   150.0

// Home position
#define HOME_ANGLE       90.0

// Channel mapping: [M0_pan, M0_tilt, M1_pan, M1_tilt, ...]
const uint8_t CHANNEL_MAP[NUM_SERVOS] = {0, 1, 2, 3, 4, 5, 6, 7};

// ─── Globals ──────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

float current_angles[NUM_SERVOS];
String serial_buffer = "";

// ─── Helper functions ─────────────────────────────────────────────────

uint16_t angleToPWM(float angle) {
    angle = constrain(angle, 0.0, 180.0);
    float ticks = SERVO_MIN_TICKS + (angle / 180.0) * (SERVO_MAX_TICKS - SERVO_MIN_TICKS);
    return (uint16_t)(ticks + 0.5);
}

void setServoAngle(uint8_t servo_idx, float angle) {
    angle = constrain(angle, SAFE_MIN_ANGLE, SAFE_MAX_ANGLE);
    uint16_t ticks = angleToPWM(angle);
    pwm.setPWM(CHANNEL_MAP[servo_idx], 0, ticks);
    current_angles[servo_idx] = angle;
}

/**
 * Parse ALL command: ALL:p0,t0,p1,t1,p2,t2,p3,t3
 * Returns true if parsed successfully, fills angles[8].
 */
bool parseAllCommand(String cmd, float angles[NUM_SERVOS]) {
    cmd.trim();

    // Handshake
    if (cmd == "PING") {
        Serial.println("PONG");
        return false;
    }

    // Check prefix
    if (!cmd.startsWith("ALL:")) {
        // Try legacy single-mirror format: PAN:xx,TILT:yy
        int panIdx = cmd.indexOf("PAN:");
        int commaIdx = cmd.indexOf(",TILT:");
        if (panIdx >= 0 && commaIdx >= 0) {
            float pan = cmd.substring(panIdx + 4, commaIdx).toFloat();
            float tilt = cmd.substring(commaIdx + 6).toFloat();
            // Broadcast to all mirrors
            for (int i = 0; i < NUM_SERVOS; i += 2) {
                angles[i] = pan;
                angles[i + 1] = tilt;
            }
            return true;
        }
        Serial.print("ERR:PARSE:");
        Serial.println(cmd);
        return false;
    }

    // Parse comma-separated values after "ALL:"
    String data = cmd.substring(4);
    int count = 0;
    int start = 0;

    for (int i = 0; i <= (int)data.length() && count < NUM_SERVOS; i++) {
        if (i == (int)data.length() || data.charAt(i) == ',') {
            if (i > start) {
                angles[count] = data.substring(start, i).toFloat();
                count++;
            }
            start = i + 1;
        }
    }

    if (count != NUM_SERVOS) {
        Serial.print("ERR:COUNT:");
        Serial.println(count);
        return false;
    }

    // Validate ranges
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (angles[i] < -1.0 || angles[i] > 181.0) {
            Serial.println("ERR:RANGE");
            return false;
        }
    }

    return true;
}

// ─── Setup ────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);

    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) {
        delay(10);
    }

    Wire.begin(I2C_SDA, I2C_SCL);

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    delay(10);

    // Home all servos
    for (int i = 0; i < NUM_SERVOS; i++) {
        setServoAngle(i, HOME_ANGLE);
    }

    delay(500);

    Serial.println("READY");
    Serial.print("OK:ALL:");
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (i > 0) Serial.print(",");
        Serial.print(current_angles[i], 1);
    }
    Serial.println();
}

// ─── Main loop ────────────────────────────────────────────────────────
void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            float angles[NUM_SERVOS];
            if (parseAllCommand(serial_buffer, angles)) {
                // Drive all servos
                for (int i = 0; i < NUM_SERVOS; i++) {
                    setServoAngle(i, angles[i]);
                }

                // Confirmation
                Serial.print("OK:ALL:");
                for (int i = 0; i < NUM_SERVOS; i++) {
                    if (i > 0) Serial.print(",");
                    Serial.print(current_angles[i], 1);
                }
                Serial.println();
            }
            serial_buffer = "";
        } else if (c != '\r') {
            serial_buffer += c;
            if (serial_buffer.length() > 128) {
                serial_buffer = "";
            }
        }
    }
}
