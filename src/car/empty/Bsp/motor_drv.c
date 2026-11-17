#include "motor_drv.h"
#include "../Drv/drv_pwm.h"
#include "ti_msp_dl_config.h"

void MotorDrv_Init(void)
{
    Drv_PWM_Init();
    DL_GPIO_initDigitalOutput(MOTOR_DIR_B_MTR_AIN1_IOMUX);
    DL_GPIO_enableOutput(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN1_PIN);
    DL_GPIO_initDigitalOutput(MOTOR_DIR_B_MTR_AIN2_IOMUX);
    DL_GPIO_enableOutput(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN2_PIN);
    DL_GPIO_initDigitalOutput(MOTOR_DIR_A_MTR_BIN1_IOMUX);
    DL_GPIO_enableOutput(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN1_PIN);
    DL_GPIO_initDigitalOutput(MOTOR_DIR_A_MTR_BIN2_IOMUX);
    DL_GPIO_enableOutput(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN2_PIN);
    
    MotorDrv_Stop();
}

void MotorDrv_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    uint16_t left_duty, right_duty;
    Motor_DirectionTypeDef left_dir, right_dir;

    /* 速度为 0 直接停 */
    if (left_speed == 0 && right_speed == 0) {
        MotorDrv_Stop();
        return;
    }
    
    if (left_speed >= 0) {
        left_dir = MOTOR_DIR_FORWARD;
        left_duty = (uint16_t)left_speed;
    } else {
        left_dir = MOTOR_DIR_BACKWARD;
        left_duty = (uint16_t)(-left_speed);
    }
    
    if (right_speed >= 0) {
        right_dir = MOTOR_DIR_FORWARD;
        right_duty = (uint16_t)right_speed;
    } else {
        right_dir = MOTOR_DIR_BACKWARD;
        right_duty = (uint16_t)(-right_speed);
    }
    
    if (left_duty > MOTOR_PWM_MAX) left_duty = MOTOR_PWM_MAX;
    if (right_duty > MOTOR_PWM_MAX) right_duty = MOTOR_PWM_MAX;

    /* 0~200（0=停, 200=满速） */
    Drv_PWM_SetDuty(PWM_MOTOR_L_INST, 0, left_duty);
    Drv_PWM_SetDuty(PWM_MOTOR_R_INST, 0, right_duty);

    MotorDrv_SetDirection(left_dir, right_dir);
}

void MotorDrv_Stop(void)
{
    Drv_PWM_SetDuty(PWM_MOTOR_L_INST, 0, 0);
    Drv_PWM_SetDuty(PWM_MOTOR_R_INST, 0, 0);

    DL_GPIO_clearPins(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN2_PIN);
}

void MotorDrv_SetDirection(Motor_DirectionTypeDef left_dir, Motor_DirectionTypeDef right_dir)
{
    if (left_dir == MOTOR_DIR_FORWARD) {
        DL_GPIO_setPins(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_DIR_B_PORT, MOTOR_DIR_B_MTR_AIN2_PIN);
    }
    
    if (right_dir == MOTOR_DIR_FORWARD) {
        DL_GPIO_setPins(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_DIR_A_PORT, MOTOR_DIR_A_MTR_BIN2_PIN);
    }
}
