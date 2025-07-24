//0723-updated 
#ifndef __TSP_MOTOR_H
#define __TSP_MOTOR_H

#include "tsp_common_headfile.h"

#define SERVO1            1
#define SERVO2            2
#define SERVO3            3
#define SERVO4            4
#define SERVO5            5
#define SERVO6            6
#define SERVO7            7
#define SERVO8            8


#define MOTOR1            1
#define MOTOR2            2
#define FORWARD           0
#define BACKWARD          1


void tsp_servo_angle(uint8_t channel, uint16_t pulse_width);
void tsp_motor_enable();
void tsp_motor_disable();
void tsp_motor_voltage(uint8_t motor_id, uint8_t dir, uint16_t duty_cycle);

#endif