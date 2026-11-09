# Hardware Wiring & Assembly Guide

## 1. Pin Connections

| Component | Component Pin | ESP32 Pin | Notes |
|---|---|---|---|
| **VL53L0X** | VIN / VCC | 3.3V (or 5V) | Check module: if module has a 3.3V voltage regulator (e.g. GY-VL53L0X), connect to 5V or 3.3V. |
| **VL53L0X** | GND | GND | Common ground with ESP32 |
| **VL53L0X** | SDA | GPIO 21 | Default ESP32 hardware I2C data |
| **VL53L0X** | SCL | GPIO 22 | Default ESP32 hardware I2C clock |
| **VL53L0X** | XSHUT | *Leave unconnected* | Pulled high by default on breakout boards |
| **VL53L0X** | GPIO1 | *Leave unconnected* | Interrupt pin, not required for continuous mode |
| **SG90 Servo** | Signal (Orange/Yellow) | GPIO 18 | PWM control signal |
| **SG90 Servo** | VCC (Red) | External 5V or VIN | **CRITICAL**: Do NOT power from ESP32 3.3V! Servo spikes cause ESP32 brownouts and resets. |
| **SG90 Servo** | GND (Brown/Black) | GND | Connect to common ESP32 & battery ground |

---

## 2. Power Supply Recommendations
- **Servo Motor Inrush Current**: An SG90 micro-servo can draw up to 500mA–800mA stall current when rapidly changing directions.
- Powering the servo directly from the ESP32's 3.3V pin will cause the ESP32 to brownout, crash, or damage the onboard linear regulator.
- **Recommended Setup**:
  - Connect a 5V USB source (power bank or 5V power supply) to `VIN` of the ESP32 and to `VCC` of the SG90 servo.
  - Ensure all ground (`GND`) pins are tied together.
  - Adding a 100µF – 470µF electrolytic capacitor across Servo `VCC` and `GND` will smooth voltage drops.

---

## 3. Physical Mounting
1. Mount the SG90 servo securely on your chassis or a steady base.
2. Fix a small 3D-printed horn or lightweight bracket onto the servo shaft.
3. Attach the VL53L0X module to the horn facing outward horizontally, perpendicular to the axis of rotation.
4. Keep the wiring flexible so the servo can sweep 0° to 180° without tension or wire tangling.
5. Zero position: Orient the servo horn so 0° corresponds to the right side (-90° relative to robot forward), 90° corresponds to straight forward, and 180° corresponds to the left side (+90° relative to robot forward).

