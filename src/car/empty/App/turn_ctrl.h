#ifndef TURN_CTRL_H
#define TURN_CTRL_H

#include <stdint.h>
#include <stdbool.h>

void TurnCtrl_Update(float target_heading, int16_t base_speed,
                     int16_t correction_limit, bool in_place,
                     int16_t min_turn_pwm, int16_t *left,
                     int16_t *right, float *error_deg);
bool TurnCtrl_IsSettled(float target_heading, float error_limit,
                        float rate_limit);

#endif
