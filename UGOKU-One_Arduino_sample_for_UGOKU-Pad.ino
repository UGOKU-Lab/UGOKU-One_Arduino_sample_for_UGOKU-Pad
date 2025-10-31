#include "UGOKU_Pad_Controller.hpp"   // Include the custom controller header for BLE handling
#include "ESP32Servo.h"               // Include the ESP32 Servo library
#include "MotorDriver.h"
#include <Wire.h>

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

// LEDs(active LOW)
#define LED_1 2
#define LED_2 4
#define LED_13 13

// FET
#define FET_23 23

// DIP switch
#define DIP_34 34   
#define DIP_35 35

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
  pinMode(FET_23, OUTPUT);  
  pinMode(DAC_25, OUTPUT);   
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_13, OUTPUT);
  pinMode(ADC_32, INPUT);
  pinMode(ADC_33, INPUT);

  // strap対策: 上電直後は必ずHIGHで“消灯”に
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_13, HIGH);

  // DIPスイッチ入力設定（入力専用ピン）
  pinMode(DIP_34, INPUT);
  pinMode(DIP_35, INPUT);

  Serial.println("Waiting for a device to connect...");
}

void onDeviceConnect() {
  Serial.println("Device connected!");
  isConnected = true;

  // Attach servos
  servo1.attach(SERVO_14, 500, 2500);
  servo2.attach(SERVO_27, 500, 2500);
}

void onDeviceDisconnect() {
  Serial.println("Device disconnected!");
  isConnected = false;

  servo1.detach();
  servo2.detach();
}

void loop() {
  if (isConnected) {
  // DIPスイッチ状態取得
  const bool invertServo = (digitalRead(DIP_34) == HIGH);
  const bool invertMotor = (digitalRead(DIP_35) == HIGH);

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
          digitalWrite(LED_1, (btn_1 == 1) ? LOW : HIGH);
          digitalWrite(LED_2, (btn_1 == 1) ? LOW : HIGH);
          digitalWrite(LED_13, (btn_1 == 1) ? LOW : HIGH);
        }

        // ch6 outputs (changed only) 
        if (btn_6 != 0xFF && btn_6 != prev_btn_6) {
          prev_btn_6 = btn_6;
          digitalWrite(FET_23, (btn_6 == 1) ? LOW : HIGH);
          digitalWrite(DAC_25, (btn_6 == 1) ? LOW : HIGH);
        }
      }
    } else if (err == cs_err) {
      Serial.println("Checksum error on incoming packet");
    } else if (err == data_err) {
      Serial.println("Incoming packet length != 19");
    }

    #if 1 // モーター独立駆動モード
      float md1 = (stick_4 / 127.5f) - 1.0f;
      float md2 = (stick_5 / 127.5f) - 1.0f;
      if (invertMotor) { md1 = -md1; md2 = -md2; }
      MotorDriver_setSpeed(MD1, md1);
      MotorDriver_setSpeed(MD2, md2);
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

    // Servo（DIPで反転）
    uint8_t s2 = stick_2;
    uint8_t s3 = stick_3;
    if (invertServo) {
      // 0-180度を中心(90)でミラー
      s2 = (s2 <= 180) ? (uint8_t)(180 - s2) : s2;
      s3 = (s3 <= 180) ? (uint8_t)(180 - s3) : s3;
    }
    servo1.write(s2);
    servo2.write(s3);

    // PSD distance (same)
    int psd = analogRead(ADC_33);
    float dist = 1 / (float)psd * 30000;  // Conversion of analogue values to cm
    int dist_int = (int)dist;
    controller.write_data(10, dist_int);
  }

  delay(50);
}
