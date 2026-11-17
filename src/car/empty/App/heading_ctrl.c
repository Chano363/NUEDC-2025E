#include "heading_ctrl.h"
#include "turn_ctrl.h"
#include "gyro_fusion.h"

/* 0°回正测试参数，实车调角度环时优先修改这里。 */
#define ZERO_HOLD_TARGET_DEG       0.0f
#define ZERO_HOLD_MAX_PWM          90
#define ZERO_HOLD_MIN_TURN_PWM     55

static bool g_zero_hold_active;

void HeadingCtrl_Hold(float target_heading, int16_t base_speed,
                      int16_t correction_limit, int16_t *left,
                      int16_t *right, float *error_deg)
{
    TurnCtrl_Update(target_heading, base_speed, correction_limit, false, 0,
                    left, right, error_deg);
}

void HeadingCtrl_TurnTo(float target_heading, int16_t base_speed,
                        int16_t correction_limit, int16_t *left,
                        int16_t *right, float *error_deg)
{
    TurnCtrl_Update(target_heading, base_speed, correction_limit, false, 0,
                    left, right, error_deg);
}

void HeadingCtrl_AlignInPlace(float target_heading, int16_t correction_limit,
                              int16_t min_turn_pwm, int16_t *left,
                              int16_t *right, float *error_deg)
{
    TurnCtrl_Update(target_heading, 0, correction_limit, true, min_turn_pwm,
                    left, right, error_deg);
}

bool HeadingCtrl_IsSettled(float target_heading, float error_limit,
                           float rate_limit)
{
    return TurnCtrl_IsSettled(target_heading, error_limit, rate_limit);
}

void HeadingCtrl_ZeroHoldStart(void)
{
    g_zero_hold_active = true;
}

void HeadingCtrl_ZeroHoldStop(void)
{
    g_zero_hold_active = false;
}

bool HeadingCtrl_ZeroHoldIsActive(void)
{
    return g_zero_hold_active;
}

void HeadingCtrl_ZeroHoldUpdate(int16_t *left, int16_t *right,
                                float *error_deg)
{
    if (!g_zero_hold_active || !GyroFusion_IsValid()) {
        *left = 0;
        *right = 0;
        if (error_deg) *error_deg = 0.0f;
        return;
    }

    HeadingCtrl_AlignInPlace(ZERO_HOLD_TARGET_DEG,
        ZERO_HOLD_MAX_PWM, ZERO_HOLD_MIN_TURN_PWM,
        left, right, error_deg);
}
