#pragma once

#include <Arduino.h>
#include <Wire.h>

// ICM-42605 only (UGOKU One V2) using Arduino ICM42605 library
// Units:
//  - Acceleration: g (gravity = 1.0)
//  - Gyro: deg/s
inline bool IMU_init(TwoWire &wire = Wire);
inline bool IMU_accelAvailable();
inline bool IMU_gyroAvailable();
inline void IMU_readAccel(float &ax, float &ay, float &az);
inline void IMU_readGyro(float &gx, float &gy, float &gz);
inline uint8_t IMU_whoAmI();
inline int IMU_readAccelGyro(float &ax, float &ay, float &az,
                             float &gx, float &gy, float &gz);
inline void IMU_readRawCounts(int16_t &ax, int16_t &ay, int16_t &az,
                              int16_t &gx, int16_t &gy, int16_t &gz);
inline bool IMU_usingCountsFallback();
inline uint8_t IMU_getI2CAddr();
// Low-level I2C helpers (bank0)
inline void ICM_writeReg(uint8_t reg, uint8_t val);
inline int  ICM_readRegs(uint8_t reg, uint8_t *buf, int len);

#include <ICM42605.h>

// The installed ICM42605 library (v1.1.0) requires constructor args (TwoWire, address)
// Default address is usually 0x68. If your board uses 0x69, we detect and use it.
#ifndef ICM42605_I2C_ADDR
#define ICM42605_I2C_ADDR 0x68
#endif
// Runtime-detected I2C address (0x68 default, 0x69 if AD0 pulled high)
static uint8_t g_icm_addr = ICM42605_I2C_ADDR;
static ICM42605* g_icm = nullptr; // not used in direct mode, kept for fallback
static bool g_initialized = false;
static bool g_directMode = true; // prefer direct register reads
static float g_accScale = 2.0f / 32768.0f;   // updated after reading ACCEL_CONFIG0
static float g_gyrScale = 2000.0f / 32768.0f; // updated after reading GYRO_CONFIG0
// Debug: force using raw counts scaling regardless of library float outputs
#ifndef IMU_FORCE_COUNTS_SCALING
#define IMU_FORCE_COUNTS_SCALING 0
#endif
// Fallback scale factors assuming FS: accel=±8g, gyro=±2000 dps
static constexpr float ACC_SCALE_G_PER_COUNT  = 2.0f / 32768.0f;    // ~0.000061035156 g/LSB (FS=±2g)
static constexpr float GYRO_SCALE_DPS_PER_COUNT = 2000.0f / 32768.0f; // ~0.061035156 dps/LSB

inline uint8_t whoAmI_at(uint8_t addr) {
  constexpr uint8_t WHO_AM_I_REG = 0x75;
  Wire.beginTransmission(addr);
  Wire.write(WHO_AM_I_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0x00;
}

inline bool IMU_init(TwoWire &wire) {
  wire.begin();
  wire.setClock(100000); // 安定優先: 100kHz
  delay(100);
  // Probe 0x68 then 0x69
  uint8_t id = whoAmI_at(0x68);
  if (!(id == 0x42 || id == 0x47)) {
    id = whoAmI_at(0x69);
    if (id == 0x42 || id == 0x47) g_icm_addr = 0x69; else g_icm_addr = 0x68; // default
  } else {
    g_icm_addr = 0x68;
  }
  // Some boards may report WHO_AM_I=0x47 (ICM-42688 compatible). Accept 0x42/0x47.
  bool id_ok = (id == 0x42 || id == 0x47);
  if (!id_ok) {
    return false;
  }
  // Create library instance only for potential fallback, but direct modeを使用
  g_icm = new ICM42605(Wire, g_icm_addr);
  int ret = g_icm->begin();
  (void)ret; // direct mode中心のためretは参考値

  // Ensure bank 0 selected and Accel/Gyro enabled in LN mode (先に電源ONしてから設定する)
  ICM_writeReg(0x76, 0x00); // REG_BANK_SEL -> bank0
  ICM_writeReg(0x4E, 0x0A); // PWR_MGMT0: Gyro=10b(LN), Accel=10b(LN)
  delay(10);
  // Disable I3C (INTF_CONFIG1 bit0=0) to force I2C-only
  // Write whole register to 0x00 (safe default for I2C)
  ICM_writeReg(0x4D, 0x00);
  delay(2);
  // Sanity: try reading accel/gyro data registers once (non-fatal)
  uint8_t sanity[12];
  int saneRead = ICM_readRegs(0x1F, sanity, 12);
  if (saneRead != 12) {
    // Try a soft reset (DEVICE_CONFIG: 0x11, bit0=1) and re-enable
    ICM_writeReg(0x11, 0x01);
    delay(50);
  ICM_writeReg(0x76, 0x00);
  ICM_writeReg(0x4E, 0x0A);
    delay(10);
  }

  // Optional: ライブラリ設定（fallback用）。直読みをメインにします。
  (void)g_icm->setAccelFS(ICM42605::gpm2);
  (void)g_icm->setGyroFS(ICM42605::dps2000);
  (void)g_icm->setAccelODR(ICM42605::odr100);
  (void)g_icm->setGyroODR(ICM42605::odr200);
  (void)g_icm->setFilters(true, true);
  (void)g_icm->enableDataReadyInterrupt();

  // Read config to determine actual FS and set scale factors
  uint8_t accCfg=0, gyrCfg=0;
  if (ICM_readRegs(0x50, &accCfg, 1) == 1) {
    uint8_t afs = (accCfg >> 2) & 0x03; // [3:2]
    float fs_g = 2.0f; // default
    switch (afs) {
      case 0: fs_g = 16.0f; break; // 00: ±16g
      case 1: fs_g = 8.0f;  break; // 01: ±8g
      case 2: fs_g = 4.0f;  break; // 10: ±4g
      case 3: fs_g = 2.0f;  break; // 11: ±2g
    }
    g_accScale = fs_g / 32768.0f;
  }
  if (ICM_readRegs(0x4F, &gyrCfg, 1) == 1) {
    uint8_t gfs = (gyrCfg >> 2) & 0x03; // [3:2]
    float fs_dps = 2000.0f; // default
    switch (gfs) {
      case 0: fs_dps = 2000.0f; break; // 00: ±2000 dps
      case 1: fs_dps = 1000.0f; break; // 01: ±1000 dps
      case 2: fs_dps = 500.0f;  break; // 10: ±500 dps
      case 3: fs_dps = 250.0f;  break; // 11: ±250 dps
    }
    g_gyrScale = fs_dps / 32768.0f;
  }

  g_initialized = true;
  return true;
}

// The hydra ICM42605 lib does not provide available() flags; we always read on demand.
inline bool IMU_accelAvailable() { return true; }
inline bool IMU_gyroAvailable()  { return true; }

inline void IMU_readAccel(float &ax, float &ay, float &az) {
  float gx, gy, gz;
  IMU_readAccelGyro(ax, ay, az, gx, gy, gz);
}

inline void IMU_readGyro(float &gx, float &gy, float &gz)  {
  float ax, ay, az;
  IMU_readAccelGyro(ax, ay, az, gx, gy, gz);
}

// hydra library has whoAmI() protected; read via I2C directly.
inline uint8_t IMU_whoAmI() { return whoAmI_at(g_icm_addr); }

inline int IMU_readAccelGyro(float &ax, float &ay, float &az,
                             float &gx, float &gy, float &gz) {
  if (!g_initialized) { ax=ay=az=0.0f; gx=gy=gz=0.0f; return -1; }
  // Read without blocking on DRDY (一部環境でDRDYが立たない事例対策)
  // 直読み（同時スナップショット）
  uint8_t buf[12];
  int n = ICM_readRegs(0x1F, buf, 12);
  if (n == 12) {
    int16_t cax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t cay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t caz = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t cgx = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t cgy = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t cgz = (int16_t)((buf[10] << 8) | buf[11]);
    // スケール（FSをレジスタから反映）
    ax = cax * g_accScale; ay = cay * g_accScale; az = caz * g_accScale;
    gx = cgx * g_gyrScale; gy = cgy * g_gyrScale; gz = cgz * g_gyrScale;
    return 0;
  }
  // フォールバック1: 分割読み（加速度6B + ジャイロ6B）
  uint8_t a6[6], g6[6];
  int na = ICM_readRegs(0x1F, a6, 6);
  int ng = ICM_readRegs(0x25, g6, 6);
  if (na == 6 && ng == 6) {
    int16_t cax = (int16_t)((a6[0] << 8) | a6[1]);
    int16_t cay = (int16_t)((a6[2] << 8) | a6[3]);
    int16_t caz = (int16_t)((a6[4] << 8) | a6[5]);
    int16_t cgx = (int16_t)((g6[0] << 8) | g6[1]);
    int16_t cgy = (int16_t)((g6[2] << 8) | g6[3]);
    int16_t cgz = (int16_t)((g6[4] << 8) | g6[5]);
    ax = cax * g_accScale; ay = cay * g_accScale; az = caz * g_accScale;
    gx = cgx * g_gyrScale; gy = cgy * g_gyrScale; gz = cgz * g_gyrScale;
    return 0;
  }
  // フォールバック2: ライブラリ経由
  if (g_icm) {
    int ret = g_icm->getAGT();
    if (ret >= 0) {
      ax = g_icm->accX(); ay = g_icm->accY(); az = g_icm->accZ();
      gx = g_icm->gyrX(); gy = g_icm->gyrY(); gz = g_icm->gyrZ();
      return ret;
    }
  }
  ax=ay=az=0.0f; gx=gy=gz=0.0f; return -1;
}

inline void IMU_readRawCounts(int16_t &ax, int16_t &ay, int16_t &az,
                              int16_t &gx, int16_t &gy, int16_t &gz) {
  if (!g_initialized || g_icm == nullptr) { ax=ay=az=gx=gy=gz=0; return; }
  g_icm->getAGT();
  ax = g_icm->getAccelX_count();
  ay = g_icm->getAccelY_count();
  az = g_icm->getAccelZ_count();
  gx = g_icm->getGyroX_count();
  gy = g_icm->getGyroY_count();
  gz = g_icm->getGyroZ_count();
}

inline bool IMU_usingCountsFallback() { return g_directMode; }
inline uint8_t IMU_getI2CAddr() { return g_icm_addr; }

inline void ICM_writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(g_icm_addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

inline int ICM_readRegs(uint8_t reg, uint8_t* buf, int len) {
  // Ensure bank0 is selected before any read
  Wire.beginTransmission(g_icm_addr);
  Wire.write((uint8_t)0x76);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(true) != 0) return -1;

  // Write target register, keep connection for repeated-start read
  Wire.beginTransmission(g_icm_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1; // no STOP
  int i = 0;
  int toRead = len;
  Wire.requestFrom((int)g_icm_addr, (int)toRead, (int)true); // send STOP after read
  while (Wire.available() && i < len) {
    buf[i++] = Wire.read();
  }
  return i;
}
