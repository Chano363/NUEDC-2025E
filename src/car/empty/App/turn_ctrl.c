#include "turn_ctrl.h"
#include "gyro_fusion.h"
#include "Bsp/motor_drv.h"
#include "Lib/math_utils.h"

/* 角度外环参数：输出为左右轮差速 PWM。 */
#define TURN_KP                 2.50f
#define TURN_KD                 0.18f
/*
 * ICM42688 当前安装方向下，正差速产生的车体角速度与航向正方向相反，
 * 因此角度环输出需要取反。
 */
#define TURN_SIGN              -1.0f
#define TURN_STOP_ERROR_DEG     0.8f
#define TURN_STOP_RATE_DPS      4.0f
#define TURN_MIN_PWM_ERROR_DEG  1.5f

static float g_last_error;

void TurnCtrl_Update(float target_heading, int16_t base_speed,
                     int16_t correction_limit, bool in_place,
                     int16_t min_turn_pwm, int16_t *left,
                     int16_t *right, float *error_deg)
{
    float error = WrapAngle180(target_heading - GyroFusion_GetHeading());

    /*
     * ±180°是环形角度的同一点。陀螺仪噪声会让误差符号来回跳变，
     * 保持上一次转向方向，避免左右轮在边界处反复换向。
     */
    if (AbsFloat(error) > 179.0f) {
        error = (g_last_error < 0.0f) ? -180.0f : 180.0f;
    }
    if (AbsFloat(error) > TURN_STOP_ERROR_DEG) {
        g_last_error = error;
    }

    float correction = TURN_SIGN
        * (TURN_KP * error - TURN_KD * GyroFusion_GetRate());
    int32_t delta = (int32_t)LimitValue(correction,
        (float)-correction_limit, (float)correction_limit);

    if (in_place
        && AbsFloat(error) <= TURN_STOP_ERROR_DEG
        && AbsFloat(GyroFusion_GetRate()) <= TURN_STOP_RATE_DPS) {
        delta = 0;
    }

    if (in_place && AbsFloat(error) >= TURN_MIN_PWM_ERROR_DEG && delta != 0) {
        int32_t magnitude = (delta < 0) ? -delta : delta;
        if (magnitude < min_turn_pwm)
            delta = (delta < 0) ? -min_turn_pwm : min_turn_pwm;
    }
    if (error_deg) *error_deg = error;
    if (in_place) {
        *left = (int16_t)LimitIntValue(delta, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
        *right = (int16_t)LimitIntValue(-delta, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    } else {
        *left = (int16_t)LimitIntValue(base_speed + delta, 10, MOTOR_PWM_MAX);
        *right = (int16_t)LimitIntValue(base_speed - delta, 10, MOTOR_PWM_MAX);
    }
}

bool TurnCtrl_IsSettled(float target_heading, float error_limit,
                        float rate_limit)
{
    return AbsFloat(WrapAngle180(target_heading - GyroFusion_GetHeading()))
        < error_limit && AbsFloat(GyroFusion_GetRate()) < rate_limit;
}
