#ifndef _H3_TASK_H
#define _H3_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    H3_STATE_IDLE = 0,
    H3_STATE_ARMED,
    H3_STATE_START_ALIGN,
    H3_STATE_STRAIGHT_AC,
    H3_STATE_ARC_CB,
    H3_STATE_TURN_BD,
    H3_STATE_STRAIGHT_BD,
    H3_STATE_ARC_DA,
    H3_STATE_ARC_EXIT_STRAIGHT,
    H3_STATE_ARC_ALIGN,
    H3_STATE_COMPLETE,
    H3_STATE_ERROR_GYRO,
    H3_STATE_ERROR_TIMEOUT
} H3_StateTypeDef;

typedef enum {
    H3_MODE_A = 0,
    H3_MODE_B,
    H3_MODE_C
} H3_ModeTypeDef;

typedef struct {
    int16_t target_left;
    int16_t target_right;
    int16_t deviation;
    float heading_error_deg;
    uint8_t digital_status;
    uint8_t lap_count;
    float target_heading_deg;
    H3_StateTypeDef state;
    bool running;
    bool finished;
} H3_OutputTypeDef;

void H3_Init(void);
void H3_SetMode(H3_ModeTypeDef mode);
H3_ModeTypeDef H3_GetMode(void);
void H3_CycleMode(void);
void H3_Start(uint32_t now_ms);
void H3_Update(uint32_t now_ms);
H3_OutputTypeDef H3_GetOutput(void);
char H3_GetStateCode(void);

#endif
