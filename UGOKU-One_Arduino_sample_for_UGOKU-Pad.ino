#include "UGOKU-Pad_Controller.hpp"   // Include the custom controller header for BLE handling
#include "ESP32Servo.h"               // Include the ESP32 Servo library
#include "MotorDriver.h"
#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>

UGOKUPadController UGOKUPad;
bool isConnected = false;

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  UGOKUPad.begin("UGOKU One V2");
  UGOKUPad.setConnectionHandlers(onConnect, onDisconnect);

  MotorDriver_begin();

  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(23, OUTPUT);  

  //LED off
  digitalWrite(2, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(13, HIGH);
  digitalWrite(23, LOW);

  Serial.println("Waiting for a device to connect...");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void onConnect() {
  Serial.println("Device connected!");
  isConnected = true;
  servo1.attach(14, 500, 2500);
  servo2.attach(27, 500, 2500);
}

void onDisconnect() {
  Serial.println("Device disconnected!");
  isConnected = false;
  servo1.detach();
  servo2.detach();
}

void loop() {
  //Disconnected: stop outputs
  if (!isConnected) return;

  //Update UGOKU Pad data
  if (!UGOKUPad.update()) return;

  // DIPスイッチ状態取得
  const bool invertServo = (digitalRead(DIP_34) == HIGH);
  const bool invertMotor = (digitalRead(DIP_35) == HIGH);

  // LED制御
  digitalWrite(2,!UGOKUPad.read(1));
  digitalWrite(4,!UGOKUPad.read(2));
  digitalWrite(13,!UGOKUPad.read(3));

  // FET制御
  digitalWrite(23,UGOKUPad.read(4));

  #if 1 // モーター独立駆動モード
    float md1 = (UGOKUPad.read(10) / 127.5f) - 1.0f;
    float md2 = (UGOKUPad.read(11) / 127.5f) - 1.0f;
    if (invertMotor) { md1 = -md1; md2 = -md2; }
    MotorDriver_setSpeed(MD1, md1);
    MotorDriver_setSpeed(MD2, md2);
  #endif

  #if 0 // モーター対向2輪1ジョイスティックモード
    float stick_x_duty = (float)UGOKUPad.read(10) / 127.5f - 1.0f;
    float stick_y_duty = (float)UGOKUPad.read(11) / 127.5f - 1.0f;

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


    float x, y, z;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);

      Serial.print(x);
      Serial.print('\t');
      Serial.print(y);
      Serial.print('\t');
      Serial.println(z);
    }
  }

    delay(50);
}
