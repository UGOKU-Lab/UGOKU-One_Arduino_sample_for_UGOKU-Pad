#include "UGOKU_Pad_Controller.hpp"   // Include the custom controller header for BLE handling
#include "ESP32Servo.h"               // Include the ESP32 Servo library
#include "MotorDriver.h"
#include <Wire.h>
#include "IMUAngleEstimator.hpp"

UGOKU_Pad_Controller controller;      // Instantiate the UGOKU Pad Controller object

// Define the pins
#define PIN_SERVO_1 14                 // Pin number for the servo
#define PIN_SERVO_2 27                 // Pin number for the servo
#define PIN_ANALOG_READ 39             // Pin number for the analog input
#define PIN_LED_1 2
#define PIN_LED_2 4
#define PIN_LED_3 23 // Pin number for the LED

Servo servo1;                         // Create a Servo object
Servo servo2;                         // Create a Servo object

bool isConnected = false;             // Boolean flag to track BLE connection status

// (kept for structure parity; not used for prints now)
uint8_t lastPrintedCh  = 255;

// Default “center” position for joystick
uint8_t stick_x = 90;
uint8_t stick_y = 90;
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
  Serial.begin(115200);               // Initialize the serial communication with a baud rate of 115200

  // Setup the BLE connection
  controller.setup("UGOKU One");      // Set the BLE device name

  // Set callback functions for when a device connects and disconnects
  controller.setOnConnectCallback(onDeviceConnect);      // Function called on device connection
  controller.setOnDisconnectCallback(onDeviceDisconnect);  // Function called on device disconnection

  MotorDriver_begin();

  // Setup the servo
  servo1.setPeriodHertz(50);          // Set the servo PWM frequency to 50Hz (typical for servos)
  servo2.setPeriodHertz(50);

  pinMode(18, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  digitalWrite(PIN_LED_1, HIGH);
  digitalWrite(PIN_LED_2, HIGH);
  digitalWrite(PIN_LED_3, HIGH);

  Serial.println("Waiting for a device to connect...");  // Print waiting message

  // Initialize IMU (always on)
  if (!imu.begin()) {
    Serial.println("BMI270 init failed");
  }
}

// Function called when a BLE device connects
void onDeviceConnect() {
  Serial.println("Device connected!");  // Print connection message
  isConnected = true;                   // Set the connection flag to true

  // Attach the servo to the defined pin, with pulse range between 500 to 2500 microseconds
  servo1.attach(PIN_SERVO_1, 500, 2500);
  servo2.attach(PIN_SERVO_2, 500, 2500);
}

// Function called when a BLE device disconnects
void onDeviceDisconnect() {
  Serial.println("Device disconnected!");  // Print disconnection message
  isConnected = false;                     // Set the connection flag to false

  servo1.detach();                       // Detach the servos to stop controlling them
  servo2.detach();
}

void loop() {
  // Only run if a device is connected via BLE
  if (isConnected) {
    uint8_t err = controller.read_data();

    if (err == no_err) {
      uint8_t pairs = controller.getLastPairsCount();

      // If there is at least one pair, find out which channels changed
      if (pairs > 0) {
        // Map channels -> variables in one place (includes ch1 & ch6)
        const uint8_t channels[] = { 1, 2, 3, 4, 5, 6 };
        uint8_t*      targets[]  = { &btn_1, &stick_x, &stick_y, &stick_4, &stick_5, &btn_6 };

        // Update all mapped channels in one pass
        for (uint8_t i = 0; i < sizeof(channels)/sizeof(channels[0]); ++i) {
          updateFromChannel(channels[i], *targets[i]);
        }

        // === ch1 LED control (changed only) ===
        if (btn_1 != 0xFF && btn_1 != prev_btn_1) {
          prev_btn_1 = btn_1;
          digitalWrite(PIN_LED_1, (btn_1 == 1) ? LOW : HIGH);
          digitalWrite(PIN_LED_2, (btn_1 == 1) ? LOW : HIGH);
          digitalWrite(PIN_LED_3, (btn_1 == 1) ? LOW : HIGH);
        }

        // === ch6 outputs (changed only) ===
        if (btn_6 != 0xFF && btn_6 != prev_btn_6) {
          prev_btn_6 = btn_6;
          digitalWrite(18, (btn_6 == 1) ? LOW : HIGH);
          digitalWrite(26, (btn_6 == 1) ? LOW : HIGH);
        }
      }
    } else if (err == cs_err) {
      Serial.println("Checksum error on incoming packet");
    } else if (err == data_err) {
      Serial.println("Incoming packet length != 19");
    }

    // Motor (same formula/方向)
    MotorDriver_setSpeed(MD1, (stick_4 / 127.5f) - 1.0f);
    MotorDriver_setSpeed(MD2, (stick_5 / 127.5f) - 1.0f);

    // Servo (same)
    servo1.write(stick_x);
    servo2.write(stick_y);

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
      // Map to 0..180 with flat=90 as requested
      ch[0] = 20; val[0] = imu.rollByte180();
      ch[1] = 21; val[1] = imu.pitchByte180();
      ch[2] = 22; val[2] = imu.yawByte180();
      controller.write_data(ch, val);
    }
  }

  delay(50);  // Add a small delay to reduce the loop frequency
}
