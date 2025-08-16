#include "IMUAngleEstimator.hpp"

#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

static inline float constrainf_(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

uint8_t IMUAngleEstimator::angleToByte(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  float v = (deg + 180.0f) * (255.0f / 360.0f);
  v = constrainf_(v, 0.0f, 255.0f);
  return (uint8_t)(v + 0.5f);
}

float IMUAngleEstimator::wrap180(float deg) {
  if (deg > 180.0f) deg -= 360.0f;
  else if (deg < -180.0f) deg += 360.0f;
  return deg;
}

uint8_t IMUAngleEstimator::mapDegToServoByte(float deg, float limitDeg) {
  if (deg > limitDeg) deg = limitDeg;
  if (deg < -limitDeg) deg = -limitDeg;
  float v = (deg + limitDeg) * (180.0f / (2.0f * limitDeg));
  if (v < 0) v = 0; if (v > 180.0f) v = 180.0f;
  return (uint8_t)(v + 0.5f);
}

uint8_t IMUAngleEstimator::rollByte180()  const { return mapDegToServoByte(_roll, 90.0f); }
uint8_t IMUAngleEstimator::pitchByte180() const { return mapDegToServoByte(_pitch, 90.0f); }
uint8_t IMUAngleEstimator::yawByte180()   const { return mapDegToServoByte(_yaw, 180.0f); }

bool IMUAngleEstimator::begin() {
  Wire.begin();
  delay(10);
  _ok = IMU.begin();
  if (!_ok) return false;

  // Seed angles with accel & mag if available
  float ax=0, ay=0, az=0;
  for (int i = 0; i < 20; ++i) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      break;
    }
    delay(5);
  }
  _roll  = atan2f(ay, az) * 180.0f / PI;
  _pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  // Do not use magnetometer (BMM150)
  _yaw = 0.0f;

  // Initialize quaternion from initial Euler
  setQuaternionFromEuler(_roll, _pitch, _yaw);
  _exInt = _eyInt = _ezInt = 0.0f;
  _lastMs = millis();
  return true;
}

void IMUAngleEstimator::update() {
  if (!_ok) return;

  uint32_t nowMs = millis();
  float dt = (nowMs - _lastMs) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f; // 1ms min to avoid zero dt
  _lastMs = nowMs;

  // Read sensors with available() gating to match Arduino IMU API
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  bool gotAcc = false, gotGyr = false;
  if (IMU.accelerationAvailable()) { IMU.readAcceleration(ax, ay, az); gotAcc = true; }
  if (IMU.gyroscopeAvailable())    { IMU.readGyroscope(gx, gy, gz);   gotGyr = true; }

  // Convert gyro to rad/s
  float gx_r = (gotGyr ? gx : 0.0f) * (PI / 180.0f);
  float gy_r = (gotGyr ? gy : 0.0f) * (PI / 180.0f);
  float gz_r = (gotGyr ? gz : 0.0f) * (PI / 180.0f);

  mahonyUpdate(gx_r, gy_r, gz_r,
               ax, ay, az, gotAcc,
               dt);

  // Quaternion -> Euler (roll X, pitch Y, yaw Z)
  float q0 = _q0, q1 = _q1, q2 = _q2, q3 = _q3;
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  _roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabsf(sinp) >= 1.0f)
    _pitch = copysignf(90.0f, sinp); // use 90 deg if out of range
  else
    _pitch = asinf(sinp) * 180.0f / PI;

  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  _yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;

  _roll  = wrap180(_roll);
  _pitch = wrap180(_pitch);
  _yaw   = wrap180(_yaw);
}

void IMUAngleEstimator::setQuaternionFromEuler(float rollDeg, float pitchDeg, float yawDeg) {
  float r = rollDeg * PI / 180.0f;
  float p = pitchDeg * PI / 180.0f;
  float y = yawDeg * PI / 180.0f;
  float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f);
  float cp = cosf(p * 0.5f), sp = sinf(p * 0.5f);
  float cy = cosf(y * 0.5f), sy = sinf(y * 0.5f);
  _q0 = cr * cp * cy + sr * sp * sy;
  _q1 = sr * cp * cy - cr * sp * sy;
  _q2 = cr * sp * cy + sr * cp * sy;
  _q3 = cr * cp * sy - sr * sp * cy;
}

void IMUAngleEstimator::mahonyUpdate(float gx, float gy, float gz,
                                     float ax, float ay, float az,
                                     bool hasAcc,
                                     float dt) {
  float q0 = _q0, q1 = _q1, q2 = _q2, q3 = _q3;

  // Normalize accelerometer
  if (hasAcc) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    // Acceptable accel magnitude range ~ [0.3g, 2.5g] to avoid dynamic spikes
    if (norm > 0.3f && norm < 2.5f) {
      float inv = 1.0f / norm; ax *= inv; ay *= inv; az *= inv;
    } else {
      hasAcc = false;
    }
  }
  // Estimated direction of gravity
  float vx = 2.0f * (q1*q3 - q0*q2);
  float vy = 2.0f * (q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Error is cross product between estimated direction and measured direction of gravity
  float ex=0, ey=0, ez=0;
  if (hasAcc) {
    ex += (ay * vz - az * vy);
    ey += (az * vx - ax * vz);
    ez += (ax * vy - ay * vx);
  }

  if (_ki > 0.0f) {
    _exInt += _ki * ex * dt;
    _eyInt += _ki * ey * dt;
    _ezInt += _ki * ez * dt;
    gx += _exInt; gy += _eyInt; gz += _ezInt;
  } else {
    _exInt = _eyInt = _ezInt = 0.0f;
  }
  gx += _kp * ex; gy += _kp * ey; gz += _kp * ez;

  // Integrate rate of change of quaternion
  float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
  float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
  float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
  float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  // Normalize quaternion
  float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (norm > 1e-6f) {
    float inv = 1.0f / norm;
    _q0 = q0 * inv; _q1 = q1 * inv; _q2 = q2 * inv; _q3 = q3 * inv;
  }
}
