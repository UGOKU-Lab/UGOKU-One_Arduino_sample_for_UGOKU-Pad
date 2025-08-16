#include "UGOKU_Pad_Controller.hpp"   // Include the custom controller header for BLE handling
#include "ESP32Servo.h"               // Include the ESP32 Servo library
#include "MotorDriver.h"

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

// Track last read channel/value to avoid flooding Serial output
uint8_t lastPrintedCh  = 255;
uint8_t lastPrintedVal = 255;
uint8_t lastPrintedVal2 = 255;

// Default “center” position for joystick
uint8_t stick_x = 90;
uint8_t stick_y = 90;
uint8_t stick_4 = 127;
uint8_t stick_5 = 127;

void setup() {
  Serial.begin(115200);               // Initialize the serial communication with a baud rate of 115200

  // Setup the BLE connection
  controller.setup("UGOKU One");       // Set the BLE device name to "My ESP32"

  // Set callback functions for when a device connects and disconnects
  controller.setOnConnectCallback(onDeviceConnect);   // Function called on device connection
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

  servo1.detach();                       // Detach the servo to stop controlling it
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
        
        uint8_t ch1Val = controller.getDataByChannel(1);
        if (ch1Val != 0xFF && ch1Val != lastPrintedVal) {
          lastPrintedVal = ch1Val;
          // Channel 1 toggles LED
          digitalWrite(PIN_LED_1, (ch1Val == 1) ? LOW : HIGH);
          digitalWrite(PIN_LED_2, (ch1Val == 1) ? LOW : HIGH);
          digitalWrite(PIN_LED_3, (ch1Val == 1) ? LOW : HIGH);
          //Serial.print("LED control (channel 1): ");
          //Serial.println((ch1Val == 1) ? "OFF" : "ON");
        }

        uint8_t ch6Val = controller.getDataByChannel(6);
        if (ch6Val != 0xFF && ch6Val != lastPrintedVal2) {
          lastPrintedVal2 = ch6Val;
          digitalWrite(18, (ch6Val == 1) ? LOW : HIGH);
          digitalWrite(26, (ch6Val == 1) ? LOW : HIGH);
        }


        uint8_t ch2Val = controller.getDataByChannel(2);
        if (ch2Val != 0xFF && ch2Val != stick_x) {
          stick_x = ch2Val;
          //Serial.print("stick_x updated: ");
          //Serial.println(stick_x);
        }

        uint8_t ch3Val = controller.getDataByChannel(3);
        if (ch3Val != 0xFF && ch3Val != stick_y) {
          stick_y = ch3Val;
          //Serial.print("stick_y updated: ");
          //Serial.println(stick_y);
        }

        uint8_t ch4Val = controller.getDataByChannel(4);
        if (ch4Val != 0xFF && ch4Val != stick_4) {
          stick_4 = ch4Val;
          //Serial.print("stick_y updated: ");
          //Serial.println(stick_y);
        }

        uint8_t ch5Val = controller.getDataByChannel(5);
        if (ch5Val != 0xFF && ch5Val != stick_5) {
          stick_5 = ch5Val;
          //Serial.print("stick_y updated: ");
          //Serial.println(stick_y);
        }

      }
    } else if (err == cs_err) {
      Serial.println("Checksum error on incoming packet");
    } else if (err == data_err) {
      Serial.println("Incoming packet length != 19");
    }

    MotorDriver_setSpeed(MD1, (stick_4 / 127.5f) - 1.0f);
    MotorDriver_setSpeed(MD2, (stick_5 / 127.5f) - 1.0f);

    servo1.write(stick_x);
    servo2.write(stick_y);

    int psd = analogRead(PIN_ANALOG_READ);
    float dist = 1 / (float)psd * 30000;  // Conversion of analogue values to cm
    int dist_int = (int)dist;
    //Serial.print("dist_int = ");
    //Serial.println(dist_int);
    controller.write_data(7,dist_int);
  }

  delay(50);  // Add a small delay to reduce the loop frequency
}
