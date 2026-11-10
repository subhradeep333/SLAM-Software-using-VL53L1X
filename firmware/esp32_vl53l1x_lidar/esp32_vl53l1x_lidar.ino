/*
 * ESP32 VL53L1X LiDAR Scanner Firmware (Up to 4-Meter Long Range)
 * -------------------------------------------------------------
 * Sweeps an SG90 / MG90S micro servo from 0° to 180° and back, capturing
 * long-range Time-of-Flight (ToF) distance measurements from a VL53L1X sensor.
 * 
 * Hardware Connections:
 *   ESP32 GPIO 21 (SDA)  <--> VL53L1X SDA
 *   ESP32 GPIO 22 (SCL)  <--> VL53L1X SCL
 *   ESP32 3.3V           <--> VL53L1X VIN (or 5V if module has onboard 3.3V LDO)
 *   ESP32 GND            <--> VL53L1X GND
 * 
 *   ESP32 GPIO 18 (PWM)  <--> SG90/MG90S Servo Signal (Orange / Yellow)
 *   External 5V / VIN    <--> Servo VCC (Red)  [Do NOT power servo from ESP32 3.3V!]
 *   ESP32 GND            <--> Servo GND (Brown) [Common Ground]
 * 
 * Libraries Required:
 *   - "VL53L1X" by Pololu
 *   - "ESP32Servo" by Kevin Harrington
 */

#include <Wire.h>
#include <VL53L1X.h>
#include <ESP32Servo.h>

// --- Configuration Constants ---
#define SERVO_PIN        18
#define I2C_SDA_PIN      21
#define I2C_SCL_PIN      22
#define SERIAL_BAUD      115200

// Sweep configuration
#define ANGLE_MIN        0     // Minimum sweep angle (degrees)
#define ANGLE_MAX        180   // Maximum sweep angle (degrees)
#define ANGLE_STEP       2     // Step resolution in degrees (91 points per sweep)
#define SERVO_SETTLE_MS  15    // Settle time for servo step (ms)

// Range limits (VL53L1X Long Range mode: up to 4000mm)
#define SENSOR_MIN_MM    40
#define SENSOR_MAX_MM    4000

VL53L1X sensor;
Servo lidarServo;

int sweepDirection = 1; // 1 = forward (0 -> 180), -1 = backward (180 -> 0)
const int TOTAL_POINTS = ((ANGLE_MAX - ANGLE_MIN) / ANGLE_STEP) + 1;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 2000);

  Serial.println("# VL53L1X 2D Long-Range LiDAR Scanner initializing...");

  // Initialize I2C bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400kHz Fast I2C

  // Initialize VL53L1X sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("# ERROR: Failed to detect and initialize VL53L1X sensor!");
    Serial.println("# Check wiring: SDA -> GPIO 21, SCL -> GPIO 22, VIN -> 3.3V/5V, GND -> GND");
    while (1) {
      delay(1000);
    }
  }

  // Set to Long Distance mode (up to 4m range)
  sensor.setDistanceMode(VL53L1X::Long);
  // Set measurement timing budget (33ms gives ~30Hz sample rate)
  sensor.setMeasurementTimingBudget(33000);
  sensor.startContinuous(33);

  // Initialize Servo
  lidarServo.setPeriodHertz(50);
  lidarServo.attach(SERVO_PIN, 500, 2400);
  lidarServo.write(ANGLE_MIN);
  delay(500);

  Serial.println("# VL53L1X LiDAR Ready (Long-Range 4m Mode). Starting continuous sweep.");
}

void loop() {
  // Frame Header: $SCAN,<direction>,<total_points>
  Serial.print("$SCAN,");
  Serial.print(sweepDirection);
  Serial.print(",");
  Serial.println(TOTAL_POINTS);

  if (sweepDirection == 1) {
    for (int angle = ANGLE_MIN; angle <= ANGLE_MAX; angle += ANGLE_STEP) {
      measureAndSend(angle);
    }
    sweepDirection = -1;
  } else {
    for (int angle = ANGLE_MAX; angle >= ANGLE_MIN; angle -= ANGLE_STEP) {
      measureAndSend(angle);
    }
    sweepDirection = 1;
  }

  // Frame Footer
  Serial.println("$END");
}

void measureAndSend(int angle) {
  lidarServo.write(angle);
  delay(SERVO_SETTLE_MS);

  // Read range in millimeters from VL53L1X
  uint16_t range_mm = sensor.read(false);

  if (sensor.timeoutOccurred() || range_mm > SENSOR_MAX_MM || range_mm < SENSOR_MIN_MM) {
    range_mm = 0; // 0 represents out-of-range / no obstacle
  }

  Serial.print(angle);
  Serial.print(",");
  Serial.println(range_mm);
}

