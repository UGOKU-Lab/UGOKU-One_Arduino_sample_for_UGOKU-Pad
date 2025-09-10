#include "UGOKU_Pad_Controller.hpp"   // Include the custom controller header for BLE handling
#include "ESP32Servo.h"               // Include the ESP32 Servo library
#include "MotorDriver.h"
#include <Wire.h>
#include "IMUAngleEstimator.hpp"

UGOKU_Pad_Controller controller;      // Instantiate the UGOKU Pad Controller object

// ==== Pin map (updated) ====
// Servos (2ch のまま運用)
#define PIN_SERVO_1      14
#define PIN_SERVO_2      27

// Analog
#define PIN_ANALOG_READ  33   // そのまま（ADC1_CH3 / 入力専用）

// LEDs (カソード=GPIO, LOWで点灯)
#define PIN_LED_1         2   // strap注意: 起動直後は必ずHIGH初期化
#define PIN_LED_2         4
#define PIN_LED_3        13   // ← 23 から 13 に変更

// btn_6でON/OFFする汎用出力（18/26 → 新マップに合わせて変更）
#define PIN_OUT_1        23   // FET_SW（ゲート）想定
#define PIN_OUT_2        25   // 予備GPIO（DAC1と排他運用に注意）

Servo servo1;
Servo servo2;

bool isConnected = false;

// (kept for structure parity; not used for prints now)
uint8_t lastPrintedCh  = 255;

// Default “center” position for joystick
uint8_t stick_2 = 90;
uint8_t stick_3 = 90;
uint8_t stick_4 = 127;
uint8_t stick_5 = 127;

// Buttons (channel 1 & 6 states)
uint8_t btn_1 = 0xFF;  // from ch1
uint8_t btn_6 = 0xFF;  // from ch6

// Track previous values for "change only" actions (same挙動)
uint8_t prev_btn_1 = 0xFF;
uint8_t prev_btn_6 = 0xFF;

IMUAngleEstimator imu;

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

  // === GPIO init (updated) ===
  pinMode(PIN_OUT_1, OUTPUT);    // 23
  pinMode(PIN_OUT_2, OUTPUT);    // 25

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  // strap対策: 上電直後は必ずHIGHで“消灯”に
  digitalWrite(PIN_LED_1, HIGH);
  digitalWrite(PIN_LED_2, HIGH);
  digitalWrite(PIN_LED_3, HIGH);

  Serial.println("Waiting for a device to connect...");

  // Initialize IMU (always on)
  if (!imu.begin()) {
    Serial.println("BMI270 init failed");
  }
}

void onDeviceConnect() {
  Serial.println("Device connected!");
  isConnected = true;

  // Attach servos
  servo1.attach(PIN_SERVO_1, 500, 2500);
  servo2.attach(PIN_SERVO_2, 500, 2500);
}

void onDeviceDisconnect() {
  Serial.println("Device disconnected!");
  isConnected = false;

  servo1.detach();
  servo2.detach();
}

void loop() {
  if (isConnected) {
    uint8_t err = controller.read_data();

    if (err == no_err) {
      uint8_t pairs = controller.getLastPairsCount();

      if (pairs > 0) {
        const uint8_t channels[] = { 1, 2, 3, 4, 5, 6 };
        uint8_t*      targets[]  = { &btn_1, &stick_2, &stick_3, &stick_4, &stick_5, &btn_6 };

        for (uint8_t i = 0; i < sizeof(channels)/sizeof(channels[0]); ++i) {
          updateFromChannel(channels[i], *targets[i]);
        }

        // ch1 LED control (changed only)
        if (btn_1 != 0xFF && btn_1 != prev_btn_1) {
          prev_btn_1 = btn_1;
          digitalWrite(PIN_LED_1, (btn_1 == 1) ? LOW : HIGH);
          digitalWrite(PIN_LED_2, (btn_1 == 1) ? LOW : HIGH);
          digitalWrite(PIN_LED_3, (btn_1 == 1) ? LOW : HIGH);
        }

        // ch6 outputs (changed only)  ← 23/25 に変更
        if (btn_6 != 0xFF && btn_6 != prev_btn_6) {
          prev_btn_6 = btn_6;
          digitalWrite(PIN_OUT_1, (btn_6 == 1) ? LOW : HIGH);
          digitalWrite(PIN_OUT_2, (btn_6 == 1) ? LOW : HIGH);
        }
      }
    } else if (err == cs_err) {
      Serial.println("Checksum error on incoming packet");
    } else if (err == data_err) {
      Serial.println("Incoming packet length != 19");
    }

    #if 1 // モーター独立駆動モード
      MotorDriver_setSpeed(MD1, (stick_4 / 127.5f) - 1.0f);
      MotorDriver_setSpeed(MD2, (stick_5 / 127.5f) - 1.0f);
    #endif

    #if 0 // モーター対向2輪1ジョイスティックモード
      float stick_x_duty = (float)stick_4 / 127.5f - 1.0f;
      float stick_y_duty = (float)stick_5 / 127.5f - 1.0f;

      float m1 = stick_x_duty + stick_y_duty;
      float m2 = stick_y_duty - stick_x_duty;

      m1 = constrain(m1, -1.0f, 1.0f);
      m2 = constrain(m2, -1.0f, 1.0f);

      MotorDriver_setSpeed(MD1, m1);
      MotorDriver_setSpeed(MD2, m2);
    #endif

    // Servo (same)
    servo1.write(stick_2);
    servo2.write(stick_3);

    // PSD distance (same)
    int psd = analogRead(PIN_ANALOG_READ);
    float dist = 1 / (float)psd * 30000;  // Conversion of analogue values to cm
    int dist_int = (int)dist;
    controller.write_data(10, dist_int);

    // IMU angles -> channels 20,21,22
    if (imu.ok()) {
      imu.update();
      uint8_t ch[9];
      uint8_t val[9];
      for (int i = 0; i < 9; ++i) { ch[i] = 0xFF; val[i] = 0; }
      ch[0] = 20; val[0] = imu.rollByte180();
      ch[1] = 21; val[1] = imu.pitchByte180();
      ch[2] = 22; val[2] = imu.yawByte180();
      controller.write_data(ch, val);
    }
  }

  delay(50);
}
