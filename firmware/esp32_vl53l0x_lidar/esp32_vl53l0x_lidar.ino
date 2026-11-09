/*
 * ESP32 VL53L0X LiDAR Scanner Firmware
 * -------------------------------------
 * Sweeps an SG90 micro servo from 0° to 180° and back, capturing
 * Time-of-Flight (ToF) distance measurements from a VL53L0X sensor.
 * 
 * Data is streamed over Serial in a structured, self-syncing protocol
 * for consumption by ROS 2 LaserScan drivers or standalone SLAM software.
 * 
 * Hardware Connections:
 *   ESP32 GPIO 21 (SDA)  <--> VL53L0X SDA
 *   ESP32 GPIO 22 (SCL)  <--> VL53L0X SCL
 *   ESP32 3.3V           <--> VL53L0X VIN (or 5V if module has 3.3V LDO)
 *   ESP32 GND            <--> VL53L0X GND
 * 
 *   ESP32 GPIO 18 (PWM)  <--> SG90 Servo Signal (Orange / Yellow)
 *   External 5V / VIN    <--> SG90 Servo VCC (Red)  [Do NOT power servo from ESP32 3.3V!]
 *   ESP32 GND            <--> SG90 Servo GND (Brown) [Common Ground]
 * 
 * Libraries Required:
 *   - "VL53L0X" by Pololu (or "Adafruit_VL53L0X")
 *   - "ESP32Servo" by Kevin Harrington (for ESP32 servo PWM control)
 */

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>

// --- Configuration Constants ---
#define SERVO_PIN        18
#define I2C_SDA_PIN      21
#define I2C_SCL_PIN      22
#define SERIAL_BAUD      115200

// Sweep configuration
#define ANGLE_MIN        0     // Minimum sweep angle (degrees)
#define ANGLE_MAX        180   // Maximum sweep angle (degrees)
#define ANGLE_STEP       2     // Step resolution in degrees (2° gives 91 points per sweep)
#define SERVO_SETTLE_MS  12    // Delay for servo to reach target position (ms)

// Range limits (VL53L0X specifications)
#define SENSOR_MIN_MM    30
#define SENSOR_MAX_MM    2000

VL53L0X sensor;
Servo lidarServo;

int currentAngle = ANGLE_MIN;
int sweepDirection = 1; // 1 = forward (0 -> 180), -1 = backward (180 -> 0)
const int TOTAL_POINTS = ((ANGLE_MAX - ANGLE_MIN) / ANGLE_STEP) + 1;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 2000); // Wait for serial connection

  Serial.println("# VL53L0X 2D LiDAR Scanner initializing...");

  // Initialize I2C bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400kHz Fast I2C

  // Initialize VL53L0X sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("# ERROR: Failed to detect and initialize VL53L0X sensor!");
    Serial.println("# Check wiring: SDA -> GPIO 21, SCL -> GPIO 22, VIN -> 3.3V/5V, GND -> GND");
    while (1) {
      delay(1000);
    }
  }

  // Configure sensor for high speed / continuous mode
  // Reduce timing budget to ~20ms - 25ms to allow rapid sweeping
  sensor.setMeasurementTimingBudget(25000); // 25ms in microseconds
  sensor.startContinuous(25);

  // Initialize Servo
  lidarServo.setPeriodHertz(50); // Standard 50Hz servo
  lidarServo.attach(SERVO_PIN, 500, 2400); // Min and max pulse width for SG90
  lidarServo.write(ANGLE_MIN);
  delay(500); // Settle at starting position

  Serial.println("# VL53L0X LiDAR Ready. Starting continuous sweep.");
}

void loop() {
  // Send scan frame header: $SCAN,<direction>,<total_points>
  // direction: 1 for 0° -> 180°, -1 for 180° -> 0°
  Serial.print("$SCAN,");
  Serial.print(sweepDirection);
  Serial.print(",");
  Serial.println(TOTAL_POINTS);

  if (sweepDirection == 1) {
    // Forward sweep: ANGLE_MIN to ANGLE_MAX
    for (int angle = ANGLE_MIN; angle <= ANGLE_MAX; angle += ANGLE_STEP) {
      measureAndSend(angle);
    }
    sweepDirection = -1;
  } else {
    // Reverse sweep: ANGLE_MAX to ANGLE_MIN
    for (int angle = ANGLE_MAX; angle >= ANGLE_MIN; angle -= ANGLE_STEP) {
      measureAndSend(angle);
    }
    sweepDirection = 1;
  }

  // Send scan frame footer
  Serial.println("$END");
}

void measureAndSend(int angle) {
  // Move servo to target angle
  lidarServo.write(angle);
  delay(SERVO_SETTLE_MS);

  // Read range in millimeters
  uint16_t range_mm = sensor.readRangeContinuousMillimeters();

  // Handle sensor timeout or invalid / out of range reads
  if (sensor.timeoutOccurred() || range_mm > SENSOR_MAX_MM || range_mm < SENSOR_MIN_MM) {
    // -1 signifies out-of-range or no reflection
    range_mm = 0; 
  }

  // Format: <angle_deg>,<distance_mm>
  Serial.print(angle);
  Serial.print(",");
  Serial.println(range_mm);
}
