#include "MotorDriver.h"

// GPIO pins for motor inputs
// MD1: IN1 = GPIO32, IN2 = GPIO33
// MD2: IN1 = GPIO5,  IN2 = GPIO13
static const int MD1_IN1 = 32;
static const int MD1_IN2 = 33;
static const int MD2_IN1 = 5;
static const int MD2_IN2 = 13;

// PWM configuration
static const int PWM_FREQ = 20000; // 20 kHz
static const int PWM_RES  = 8;     // 8-bit (0–255)

// LEDC channels for forward/reverse per motor
static const int CH_MD1_FWD = 4;
static const int CH_MD1_REV = 5;
static const int CH_MD2_FWD = 6;
static const int CH_MD2_REV = 7;

void MotorDriver_begin() {
    // Attach and configure pins with specified channel, freq, resolution
    ledcAttachChannel(MD1_IN1, PWM_FREQ, PWM_RES, CH_MD1_FWD);
    ledcAttachChannel(MD1_IN2, PWM_FREQ, PWM_RES, CH_MD1_REV);
    ledcAttachChannel(MD2_IN1, PWM_FREQ, PWM_RES, CH_MD2_FWD);
    ledcAttachChannel(MD2_IN2, PWM_FREQ, PWM_RES, CH_MD2_REV);

    // Ensure initial brake state: both HIGH
    digitalWrite(MD1_IN1, HIGH);
    digitalWrite(MD1_IN2, HIGH);
    digitalWrite(MD2_IN1, HIGH);
    digitalWrite(MD2_IN2, HIGH);
}

void MotorDriver_setSpeed(MotorCh ch, float dutyRatio) {
    // Clamp input to -1.0 … +1.0
    dutyRatio = constrain(dutyRatio, -1.0f, 1.0f);

    // Map absolute input [0…1] → [MIN_DUTY…MAX_DUTY], keep sign
    float absIn = fabs(dutyRatio);
    float mapped = (absIn > 0.0f)
        ? (MIN_DUTY + (MAX_DUTY - MIN_DUTY) * absIn)
        : 0.0f;
    float finalRatio = (dutyRatio >= 0.0f) ? mapped : -mapped;

    // Compute PWM duty (0–255)
    uint8_t duty = (uint8_t)(fabs(finalRatio) * 255.0f);

    // Select LEDC channel numbers
    int chFwd = (ch == MD1) ? CH_MD1_FWD : CH_MD2_FWD;
    int chRev = (ch == MD1) ? CH_MD1_REV : CH_MD2_REV;

    if (duty == 0) {
        // Brake mode: detach PWM, drive both pins HIGH
        ledcDetach(MD1_IN1);
        ledcDetach(MD1_IN2);
        ledcDetach(MD2_IN1);
        ledcDetach(MD2_IN2);
        digitalWrite((ch == MD1 ? MD1_IN1 : MD2_IN1), HIGH);
        digitalWrite((ch == MD1 ? MD1_IN2 : MD2_IN2), HIGH);
    }
    else if (finalRatio > 0.0f) {
        // Forward: PWM on IN1, IN2=LOW
        ledcWriteChannel(chFwd, duty);
        ledcWriteChannel(chRev, 0);
    }
    else {
        // Reverse: IN1=LOW, PWM on IN2
        ledcWriteChannel(chFwd, 0);
        ledcWriteChannel(chRev, duty);
    }
}