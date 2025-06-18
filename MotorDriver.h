#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// Minimum and maximum duty ratio (0.0 – 1.0)
#define MIN_DUTY 0.2f
#define MAX_DUTY 1.0f

// Motor channel selection
enum MotorCh {
    MD1 = 1,
    MD2 = 2
};

// Initialize motor driver pins and PWM channels using new LEDC API
void MotorDriver_begin();

// Set motor duty ratio in range -1.0 … +1.0
//   dutyRatio <  0.0 : reverse
//   dutyRatio =  0.0 : stop (brake)
//   dutyRatio >  0.0 : forward
void MotorDriver_setSpeed(MotorCh ch, float dutyRatio);

#endif // MOTOR_DRIVER_H