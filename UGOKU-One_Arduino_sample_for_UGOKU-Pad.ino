#include "UGOKU-Pad_Controller.h"   
#include "MotorDriver.h"
#include "ESP32Servo.h"               
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

  // モーターの入力値を初期化
  UGOKUPad.write(19, 127);
  UGOKUPad.write(17, 127);

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
  const bool invertServo = (digitalRead(34) == HIGH);
  const bool invertMotor = (digitalRead(35) == HIGH);

  // LED制御
  digitalWrite(2,!UGOKUPad.read(2));
  digitalWrite(4,!UGOKUPad.read(4));
  digitalWrite(13,!UGOKUPad.read(13));

  // FET制御
  digitalWrite(23,UGOKUPad.read(23));

  #if 1 // モーター独立駆動モード
    float md1 = (UGOKUPad.read(19) / 127.0f) - 1.0f;
    float md2 = (UGOKUPad.read(17) / 127.0f) - 1.0f;
    if (invertMotor) { md1 = -md1; md2 = -md2; }
    Motor(md1, md2);
  #endif

  #if 0 // モーター対向2輪1ジョイスティックモード
    float stick_x_duty = (float)UGOKUPad.read(19) / 127.0f - 1.0f;
    float stick_y_duty = (float)UGOKUPad.read(17) / 127.0f - 1.0f;

    float m1 = stick_x_duty + stick_y_duty;
    float m2 = stick_y_duty - stick_x_duty;

    m1 = constrain(m1, -1.0f, 1.0f);
    m2 = constrain(m2, -1.0f, 1.0f);

    Motor(m1, m2);
  #endif

  // Servo（DIPで反転）
  uint8_t s2 = UGOKUPad.read(14);
  uint8_t s3 = UGOKUPad.read(27);
  if (invertServo) {
    s2 = (uint8_t)(180 - s2);
    s3 = (uint8_t)(180 - s3);
  }
  servo1.write(s2);
  servo2.write(s3);


  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    UGOKUPad.write(100, fabsf(x * 100));
    UGOKUPad.write(101, fabsf(y * 100));
    UGOKUPad.write(102, fabsf(z * 100));
  }
  
  delay(50);
}
  
