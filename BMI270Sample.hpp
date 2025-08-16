#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>

// Simple helper to initialize and read BMI270 (and BMM150 if present)
class BMI270Sample {
public:
  // Initialize I2C and IMU. Returns true on success.
  bool begin(TwoWire &wire = Wire) {
    _wire = &wire;
    _wire->begin();
    delay(10);
    // Try to initialize the Arduino BMI270/BMM150 IMU wrapper
    if (!IMU.begin()) {
      return false;
    }
    _initialized = true;
    return true;
  }

  // Read available accel/gyro values. Returns true if any data was read.
  bool read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    if (!_initialized) return false;

    bool got = false;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      got = true;
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      got = true;
    }
    return got;
  }

private:
  TwoWire *_wire = nullptr;
  bool _initialized = false;
};
