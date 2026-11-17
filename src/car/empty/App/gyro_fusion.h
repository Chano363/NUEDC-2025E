#ifndef _GYRO_FUSION_H
#define _GYRO_FUSION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float angle_deg;
    float dps;
    float fused_angle;
    float heading;
    uint32_t frame_count;
    uint32_t age_ms;
    bool valid;
} GyroFusion_DataDef;

void GyroFusion_Init(void);
void GyroFusion_Update(void);
void GyroFusion_Zero(void);
float GyroFusion_GetAngle(void);
float GyroFusion_GetHeading(void);
float GyroFusion_GetRate(void);
bool GyroFusion_IsValid(void);
GyroFusion_DataDef GyroFusion_GetData(void);

#endif
