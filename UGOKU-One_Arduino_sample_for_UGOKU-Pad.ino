#include "UGOKU_Pad_Controller.hpp"   // Include the custom controller header for BLE handling
#include "ESP32Servo.h"               // Include the ESP32 Servo library
#include "MotorDriver.h"
#include <Wire.h>
#include "IMUProvider.hpp"
#include "SwitchUtils.hpp"

UGOKU_Pad_Controller controller;      // Instantiate the UGOKU Pad Controller object

// ==== Pin map ====
// Servos
#define SERVO_14 14
#define SERVO_27 27
#define SERVO_15 15
#define SERVO_26 26

// Analog
#define ADC_32 32
#define ADC_33 33
#define DAC_25 25
#define DAC_26 26

// LEDs (カソード=GPIO, LOWで点灯)
#define LED_1 2
#define LED_2 4
#define LED_3 13

// FET
#define FET_23 23

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

bool isConnected = false;

// (kept for structure parity; not used for prints now)
uint8_t lastPrintedCh  = 255;

// Default “center” position for joystick
uint8_t stick_10 = 90;
uint8_t stick_11 = 90;
uint8_t stick_12 = 90;
uint8_t stick_13 = 90;
uint8_t stick_14 = 127;
uint8_t stick_15 = 127;

// Buttons (channel 1 & 6 states)
uint8_t SW1 = 0xFF;  // from ch1
uint8_t SW2 = 0xFF;  // from ch2
uint8_t SW3 = 0xFF;  // from ch3
uint8_t SW4 = 0xFF;  // from ch4
uint8_t SW5 = 0xFF;  // from ch5
uint8_t SW6 = 0xFF;  // from ch6
uint8_t SW7 = 0xFF;  // from ch7
uint8_t SW8 = 0xFF;  // from ch8
uint8_t SW9 = 0xFF;  // from ch9

// Track previous values for "change only" actions (same挙動)
uint8_t prev_SW1 = 0xFF;
uint8_t prev_SW2 = 0xFF;
uint8_t prev_SW3 = 0xFF;
uint8_t prev_SW4 = 0xFF;
uint8_t prev_SW5 = 0xFF;
uint8_t prev_SW6 = 0xFF;
uint8_t prev_SW7 = 0xFF;
uint8_t prev_SW8 = 0xFF;
uint8_t prev_SW9 = 0xFF;



// Simple IMU presence flag
bool imu_ok = false;

// Helper: read once and update target var if value is valid and changed
static inline void updateFromChannel(uint8_t ch, uint8_t &var) {
  uint8_t v = controller.getDataByChannel(ch);
  if (v != 0xFF && v != var) {
    var = v;
    // Serial.printf("ch%u -> %u\n", ch, var);  // debug if needed
  }
}

void setup() {
  Serial.begin(115200);

  controller.setup("UGOKU One V2");
  controller.setOnConnectCallback(onDeviceConnect);
  controller.setOnDisconnectCallback(onDeviceDisconnect);

  MotorDriver_begin();

  // Setup the servo
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);

  // === GPIO init ===
  // Digital outputs
  pinMode(FET_23, OUTPUT);   // FET (23)
  pinMode(DAC_25, OUTPUT);   // use as digital out
  pinMode(DAC_26, OUTPUT);   // use as digital out

  // LEDs (カソード=GPIO, LOWで点灯) → 初期は消灯(HIGH)
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);

  // If SW2 drives SERVO_14 as a plain digital output, set as OUTPUT
  pinMode(SERVO_14, OUTPUT);
  pinMode(SERVO_15, OUTPUT);
  pinMode(SERVO_26, OUTPUT);
  pinMode(SERVO_27, OUTPUT);


  // Analog input
  pinMode(ADC_32, INPUT);

  Serial.println("Waiting for a device to connect...");

  // Initialize IMU (ICM-42605 on UGOKU One V2)
  imu_ok = IMU_init(Wire);
  uint8_t who = IMU_whoAmI();
  Serial.printf("IMU init %s, WHO_AM_I=0x%02X, addr=0x%02X, rawPath=%s\n",
                imu_ok ? "OK" : "FAILED",
                who,
                IMU_getI2CAddr(),
                IMU_usingCountsFallback() ? "ON" : "OFF");
  // I2C scan (quick)
  Serial.println("I2C scan start...");
  for (uint8_t addr = 8; addr < 120; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" - Found device at 0x%02X\n", addr);
    }
  }
  Serial.println("I2C scan end.");
  // Dump key registers once
  if (imu_ok) {
    uint8_t v;
    if (ICM_readRegs(0x4E, &v, 1)==1) Serial.printf("PWR_MGMT0(0x4E)=0x%02X\n", v);
    if (ICM_readRegs(0x4D, &v, 1)==1) Serial.printf("INTF_CONFIG1(0x4D)=0x%02X\n", v);
    if (ICM_readRegs(0x4F, &v, 1)==1) Serial.printf("GYRO_CONFIG0(0x4F)=0x%02X\n", v);
    if (ICM_readRegs(0x50, &v, 1)==1) Serial.printf("ACCEL_CONFIG0(0x50)=0x%02X\n", v);
    if (ICM_readRegs(0x1A, &v, 1)==1) Serial.printf("INT_STATUS1(0x1A)=0x%02X\n", v);
    uint8_t data[12];
    int n = ICM_readRegs(0x1F, data, 12);
    Serial.printf("First data read len=%d, bytes=", n);
    for (int i=0;i<n;i++){ Serial.printf("%02X ", data[i]); }
    Serial.println();
  }
}

void onDeviceConnect() {
  Serial.println("Device connected!");
  isConnected = true;

  // Attach servos
  servo1.attach(SERVO_14, 500, 2500);
  servo2.attach(SERVO_27, 500, 2500);
  servo3.attach(SERVO_15, 500, 2500);
  servo4.attach(SERVO_26, 500, 2500);
}

void onDeviceDisconnect() {
  Serial.println("Device disconnected!");
  isConnected = false;

  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
}

void loop() {
  if (isConnected) {
    uint8_t err = controller.read_data();

    if (err == no_err) {
      uint8_t pairs = controller.getLastPairsCount();

      if (pairs > 0) {
        const uint8_t channels[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
        uint8_t*      targets[]  = { &SW1, &SW2, &SW3, &SW4, &SW5, &SW6, &SW7, &SW8, &SW9, &stick_10, &stick_11, &stick_12, &stick_13, &stick_14, &stick_15 };

        for (uint8_t i = 0; i < sizeof(channels)/sizeof(channels[0]); ++i) {
          updateFromChannel(channels[i], *targets[i]);
        }

        // 使い方: applySwitchPins(SW変数, ピン..., ...)
        applySwitchPins(SW1, LED_1, LED_2, LED_3);
        //applySwitchPins(SW2, SERVO_15);
        applySwitchPins(SW3, DAC_26);
        applySwitchPins(SW4, DAC_25);
        applySwitchPins(SW9, FET_23);

      }
    } else if (err == cs_err) {
      Serial.println("Checksum error on incoming packet");
    } else if (err == data_err) {
      Serial.println("Incoming packet length != 19");
    }

    #if 1 // モーター独立駆動モード
      MotorDriver_setSpeed(MD1, (stick_14 / 127.5f) - 1.0f);
      MotorDriver_setSpeed(MD2, (stick_15 / 127.5f) - 1.0f);
    #endif

    #if 0 // モーター対向2輪1ジョイスティックモード
      float stick_x_duty = (float)stick_14 / 127.5f - 1.0f;
      float stick_y_duty = (float)stick_15 / 127.5f - 1.0f;

      float m1 = stick_x_duty + stick_y_duty;
      float m2 = stick_y_duty - stick_x_duty;

      m1 = constrain(m1, -1.0f, 1.0f);
      m2 = constrain(m2, -1.0f, 1.0f);

      MotorDriver_setSpeed(MD1, m1);
      MotorDriver_setSpeed(MD2, m2);
    #endif

    // Servo (same)
    servo1.write(stick_10);
    servo2.write(stick_11);
    servo3.write(stick_12);
    servo4.write(stick_13);

    
  // PSD distance (prepare; not sent in the IMU packet to keep 9 slots)
  int psd = analogRead(ADC_32);
  float dist = 1 / (float)psd * 30000;  // Conversion of analogue values to cm
  int dist_int = (int)dist;
  if (dist_int < 0) dist_int = 0; if (dist_int > 255) dist_int = 255; // clamp to 0..255

    // Accel (30..32) + Gyro (40..42) -> one atomic packet (6 pairs used)
    {
      static uint32_t lastImuSendMs = 0;
      uint32_t nowMs = millis();
      if (nowMs - lastImuSendMs < 50) {
        // Limit to ~20 Hz to reduce jitter and BLE contention
        // Still send other channels in this loop
      } else {
        lastImuSendMs = nowMs;
        uint8_t ch[9];
        uint8_t val[9];
        for (int i = 0; i < 9; ++i) { ch[i] = 0; val[i] = 0; }
  // Read raw accel/gyro once
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  int imuRet = IMU_readAccelGyro(ax, ay, az, gx, gy, gz);

        auto mapCentered180 = [](float v, float limit) -> uint8_t {
          if (v > limit) v = limit; if (v < -limit) v = -limit;
          float s = (v + limit) * (180.0f / (2.0f * limit));
          if (s < 0) s = 0; if (s > 180.0f) s = 180.0f;
          return (uint8_t)(s + 0.5f);
        };

        // ch30..32: accel[g] mapped to 0..180 with center 90
        ch[0] = 30; ch[1] = 31; ch[2] = 32;
        val[0] = mapCentered180(ax, 2.0f); // [-2g, 2g]
        val[1] = mapCentered180(ay, 2.0f);
        val[2] = mapCentered180(az, 2.0f);

        // ch40..42: gyro[dps] mapped to 0..180 with center 90
        ch[3] = 40; ch[4] = 41; ch[5] = 42;
        val[3] = mapCentered180(gx, 250.0f); // [-250, 250] dps
        val[4] = mapCentered180(gy, 250.0f);
        val[5] = mapCentered180(gz, 250.0f);

  controller.write_data(ch, val);

        // Serial log at the same cycle (what we just sent)
        Serial.printf("TX ch30-32(acc)=%3u,%3u,%3u | ch40-42(gyro)=%3u,%3u,%3u\n",
                      val[0], val[1], val[2],
                      val[3], val[4], val[5]);
        if (imuRet < 0) {
          static uint32_t lastErrMs = 0;
          if (millis() - lastErrMs > 2000) {
            lastErrMs = millis();
            Serial.println("IMU read failed (accel/gyro registers) -> sending center (90)");
          }
        } else if (ax==0.0f && ay==0.0f && az==0.0f && gx==0.0f && gy==0.0f && gz==0.0f) {
          static uint32_t lastZeroMs = 0;
          if (millis() - lastZeroMs > 2000) {
            lastZeroMs = millis();
            Serial.println("IMU returned all zeros (sensors may be off). Check PWR_MGMT0/ODR.");
          }
        }
      }
    }
  }
  delay(50);
}
