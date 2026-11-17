#ifndef HEADING_CTRL_H
#define HEADING_CTRL_H

#include <stdint.h>
#include <stdbool.h>

void HeadingCtrl_Hold(float target_heading, int16_t base_speed,
                      int16_t correction_limit, int16_t *left,
                      int16_t *right, float *error_deg);
void HeadingCtrl_TurnTo(float target_heading, int16_t base_speed,
                        int16_t correction_limit, int16_t *left,
                        int16_t *right, float *error_deg);
void HeadingCtrl_AlignInPlace(float target_heading, int16_t correction_limit,
                              int16_t min_turn_pwm, int16_t *left,
                              int16_t *right, float *error_deg);
bool HeadingCtrl_IsSettled(float target_heading, float error_limit,
                           float rate_limit);

/* KEY3 使用的独立 0°原地回正测试。 */
void HeadingCtrl_ZeroHoldStart(void);
void HeadingCtrl_ZeroHoldStop(void);
bool HeadingCtrl_ZeroHoldIsActive(void);
void HeadingCtrl_ZeroHoldUpdate(int16_t *left, int16_t *right,
                                float *error_deg);

#endif
