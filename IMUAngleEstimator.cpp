#include "IMUAngleEstimator.hpp"

#include <Wire.h>
#include "IMUProvider.hpp"
#include <math.h>

static inline float constrainf_(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// ===== Board orientation (adjust to match sensor mounting) =====
// If roll/pitchが入れ替わって見える場合は XY をスワップ。
// 必要なら軸の正負も切り替え。
// UGOKU One V2 でICM42605がX/Y入替に見えるとのことなので既定=1。
#ifndef BOARD_SWAP_XY
#define BOARD_SWAP_XY 1
#endif
#ifndef BOARD_INVERT_X
#define BOARD_INVERT_X 0
#endif
#ifndef BOARD_INVERT_Y
#define BOARD_INVERT_Y 0
#endif
#ifndef BOARD_INVERT_Z
#define BOARD_INVERT_Z 0
#endif

static void applyBoardOrientation(float &ax, float &ay, float &az,
                                  float &gx, float &gy, float &gz) {
  // swap X <-> Y if configured
  #if BOARD_SWAP_XY
    float t = ax; ax = ay; ay = t;
    t = gx; gx = gy; gy = t;
  #endif
  // optional sign flips
  #if BOARD_INVERT_X
    ax = -ax; gx = -gx;
  #endif
  #if BOARD_INVERT_Y
    ay = -ay; gy = -gy;
  #endif
  #if BOARD_INVERT_Z
    az = -az; gz = -gz;
  #endif
}

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

uint8_t IMUAngleEstimator::rollByte180()  const { return mapDegToServoByte(_froll, 90.0f); }
uint8_t IMUAngleEstimator::pitchByte180() const { return mapDegToServoByte(_fpitch, 90.0f); }
uint8_t IMUAngleEstimator::yawByte180()   const { return mapDegToServoByte(_fyaw, 180.0f); }

bool IMUAngleEstimator::begin() {
  _ok = IMU_init(Wire);
  if (!_ok) return false;

  // Seed angles with accel & mag if available
  float ax=0, ay=0, az=0;
  for (int i = 0; i < 20; ++i) {
    if (IMU_accelAvailable()) {
      IMU_readAccel(ax, ay, az);
      break;
    }
    delay(5);
  }
  {
    // Apply board orientation to accel before initial Euler seed
    float gx=0, gy=0, gz=0; // not used for seed, but required by helper
    applyBoardOrientation(ax, ay, az, gx, gy, gz);
  }
  _roll  = atan2f(ay, az) * 180.0f / PI;
  _pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  // Yawはマグなし（重力のみ）だと相対値になります
  _yaw = 0.0f;

  // Initialize quaternion from initial Euler
  setQuaternionFromEuler(_roll, _pitch, _yaw);
  _exInt = _eyInt = _ezInt = 0.0f;
  _lastMs = millis();

  // Initial filter outputs
  _froll = _roll; _fpitch = _pitch; _fyaw = _yaw;

  // Quick gyro bias calibration (assumes device is stationary at power-up)
  // Collect ~200ms of samples
  const uint32_t calibStart = millis();
  int n = 0;
  double sumx = 0, sumy = 0, sumz = 0;
  while (millis() - calibStart < 200) {
    float ax2, ay2, az2, gx2, gy2, gz2;
    if (IMU_gyroAvailable()) {
      IMU_readGyro(gx2, gy2, gz2);
      // Orientation mapping to keep consistency
      float ax_dummy=0, ay_dummy=0, az_dummy=0;
      applyBoardOrientation(ax_dummy, ay_dummy, az_dummy, gx2, gy2, gz2);
      sumx += gx2; sumy += gy2; sumz += gz2; ++n;
    }
    delay(5);
  }
  if (n > 10) {
    _gxb = (float)(sumx / n);
    _gyb = (float)(sumy / n);
    _gzb = (float)(sumz / n);
    _biasCalibrated = true;
  }
  return true;
}

void IMUAngleEstimator::update() {
  if (!_ok) return;

  uint32_t nowMs = millis();
  float dt = (nowMs - _lastMs) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f; // 1ms min to avoid zero dt
  if (dt > kMaxDt) dt = kMaxDt; // clamp to avoid huge jumps after stalls
  _lastMs = nowMs;

  // Read sensors once (synchronized acc+gyro)
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  bool gotAcc = false, gotGyr = false;
  int ret = IMU_readAccelGyro(ax, ay, az, gx, gy, gz);
  if (ret >= 0) { gotAcc = true; gotGyr = true; }

  // Apply board orientation mapping
  applyBoardOrientation(ax, ay, az, gx, gy, gz);

  // Convert gyro to rad/s
  if (gotGyr && _biasCalibrated) {
    gx -= _gxb; gy -= _gyb; gz -= _gzb;
  }
  float gx_r = (gotGyr ? gx : 0.0f) * (PI / 180.0f);
  float gy_r = (gotGyr ? gy : 0.0f) * (PI / 180.0f);
  float gz_r = (gotGyr ? gz : 0.0f) * (PI / 180.0f);

  // Smooth accel for fusion and reject large spikes
  if (!_accInit) { _sax = ax; _say = ay; _saz = az; _accInit = true; }
  float dax = ax - _sax, day = ay - _say, daz = az - _saz;
  if (fabsf(dax) < kAccSpikeThreshG) _sax += kAccLpfAlpha * dax;
  if (fabsf(day) < kAccSpikeThreshG) _say += kAccLpfAlpha * day;
  if (fabsf(daz) < kAccSpikeThreshG) _saz += kAccLpfAlpha * daz;

  // Dynamic Kp: lower gains when gyro indicates near-static to reduce jitter
  float gabs = fabsf(gx) + fabsf(gy) + fabsf(gz);
  float kp_save = _kp;
  if (gabs < 2.0f) { // ~<2 dps total
    _kp = 1.0f;
  }

  mahonyUpdate(gx_r, gy_r, gz_r,
               _sax, _say, _saz, gotAcc,
               dt);

  _kp = kp_save;

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

  // Smooth the outputs for UI stability (simple low-pass per angle)
  _froll  = _froll  + kAngleLpfAlpha * (_roll  - _froll);
  _fpitch = _fpitch + kAngleLpfAlpha * (_pitch - _fpitch);
  _fyaw   = _fyaw   + kAngleLpfAlpha * (_yaw   - _fyaw);

  // Near-static override: use accel-only tilt to lock roll/pitch when still
  float anorm = sqrtf(_sax*_sax + _say*_say + _saz*_saz);
  float gsum = fabsf(gx) + fabsf(gy) + fabsf(gz);
  if (gsum < 1.0f && anorm > 0.9f && anorm < 1.1f) {
    float r_acc  = atan2f(_say, _saz) * 180.0f / PI;
    float p_acc  = atan2f(-_sax, sqrtf(_say * _say + _saz * _saz)) * 180.0f / PI;
    r_acc = wrap180(r_acc); p_acc = wrap180(p_acc);
    _froll  = _froll  + kAngleLpfAlpha * (r_acc - _froll);
    _fpitch = _fpitch + kAngleLpfAlpha * (p_acc - _fpitch);
  }

  // NaN/Inf guard
  if (!isfinite(_froll))  _froll = 0;
  if (!isfinite(_fpitch)) _fpitch = 0;
  if (!isfinite(_fyaw))   _fyaw = 0;
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
    // Acceptable accel magnitude range ~ [0.6g, 1.4g] to avoid dynamics and noise
    if (norm > 0.6f && norm < 1.4f) {
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
    // IMU variant: gravity alone cannot observe yaw. Don't correct yaw from accel.
    ez = 0.0f;
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
  if (norm > 1e-6f && isfinite(norm)) {
    float inv = 1.0f / norm;
    _q0 = q0 * inv; _q1 = q1 * inv; _q2 = q2 * inv; _q3 = q3 * inv;
  } else {
    // Recovery: reset quaternion from current smoothed Euler if something blew up
    setQuaternionFromEuler(_froll, _fpitch, _fyaw);
  }
}
