#pragma once

#include <Arduino.h>

// IMU backend is selected via IMUProvider.hpp; no direct includes here

class IMUAngleEstimator {
public:
  // Initialize I2C and IMU; returns true if IMU is available
  bool begin();

  // Update internal roll/pitch/yaw using complementary filter
  void update();

  // Degrees getters
  float rollDeg() const { return _roll; }
  float pitchDeg() const { return _pitch; }
  float yawDeg() const { return _yaw; }

  // 0..255 mapping of [-180,180] deg
  // Legacy 0..255 mapping (unused now)
  uint8_t rollByte() const { return angleToByte(_roll); }
  uint8_t pitchByte() const { return angleToByte(_pitch); }
  uint8_t yawByte() const { return angleToByte(_yaw); }

  // 0..180 mapping with flat=90
  uint8_t rollByte180() const;  // clamp [-90,90] -> [0,180]
  uint8_t pitchByte180() const; // clamp [-90,90] -> [0,180]
  uint8_t yawByte180() const;   // wrap/clamp [-180,180] -> [0,180]

  bool ok() const { return _ok; }

private:
  static uint8_t angleToByte(float deg);
  static float wrap180(float deg);
  static uint8_t mapDegToServoByte(float deg, float limitDeg);
  void setQuaternionFromEuler(float rollDeg, float pitchDeg, float yawDeg);
  void mahonyUpdate(float gx, float gy, float gz, // rad/s
                    float ax, float ay, float az, // g or any, will be normalized
                    bool hasAcc,
                    float dt);

  bool _ok = false;
  float _roll = 0.0f;
  float _pitch = 0.0f;
  float _yaw = 0.0f;
  // Smoothed angles for UI stability
  float _froll = 0.0f;
  float _fpitch = 0.0f;
  float _fyaw = 0.0f;
  uint32_t _lastMs = 0;

  // Mahony AHRS state
  float _q0 = 1.0f, _q1 = 0.0f, _q2 = 0.0f, _q3 = 0.0f; // quaternion
  float _exInt = 0.0f, _eyInt = 0.0f, _ezInt = 0.0f;     // integral error
  float _kp = 2.0f;  // proportional gain (lower -> more stable at rest)
  float _ki = 0.0f;  // integral gain (0 to disable)

  // Gyro bias (auto-calibrated at startup while stationary)
  float _gxb = 0.0f, _gyb = 0.0f, _gzb = 0.0f; // deg/s offsets
  bool  _biasCalibrated = false;
  
  // Output smoothing
  static constexpr float kAngleLpfAlpha = 0.15f; // 0..1 (higher=faster)
  static constexpr float kMaxDt = 0.2f;          // avoid huge dt on stalls

  // Accel smoothing for fusion (reduces flicker at rest)
  bool  _accInit = false;
  float _sax = 0.0f, _say = 0.0f, _saz = 0.0f;
  static constexpr float kAccLpfAlpha = 0.2f;
  static constexpr float kAccSpikeThreshG = 1.5f; // ignore sudden >1.5g diffs
};
