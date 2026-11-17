#ifndef _MOTOR_DRV_H
#define _MOTOR_DRV_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_PWM_MAX  150

typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD
} Motor_DirectionTypeDef;

void MotorDrv_Init(void);
void MotorDrv_SetSpeed(int16_t left_speed, int16_t right_speed);
void MotorDrv_SetDirection(Motor_DirectionTypeDef left_dir, Motor_DirectionTypeDef right_dir);
void MotorDrv_Stop(void);

#endif
