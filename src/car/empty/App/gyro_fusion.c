#include "gyro_fusion.h"
#include "../Bsp/bsp_init.h"
#include "../Bsp/gyro_uart.h"
#include "../Lib/math_utils.h"

#define GYRO_DATA_TIMEOUT_MS 3000U
#define GYRO_DIRECTION_SIGN (1.0f)
#define GYRO_RATE_FALLBACK_MIN_DPS 3.0f
#define GYRO_RATE_FALLBACK_MIN_DELTA_DEG 0.05f
#define GYRO_DT_MAX_MS 100U

static GyroFusion_DataDef gyro_data;
static float last_raw_angle;
static uint32_t last_frame;
static uint32_t last_frame_tick;
static bool zero_locked;

static void gyro_lock_zero(const GyroData_t *raw)
{
    last_raw_angle = raw->angle_deg;
    gyro_data.angle_deg = GYRO_DIRECTION_SIGN * raw->angle_deg;
    gyro_data.dps = GYRO_DIRECTION_SIGN * raw->dps;
    gyro_data.fused_angle = 0.0f;
    gyro_data.heading = 0.0f;
    zero_locked = true;
}

void GyroFusion_Init(void)
{
    gyro_data.angle_deg = 0.0f;
    gyro_data.dps = 0.0f;
    gyro_data.fused_angle = 0.0f;
    gyro_data.heading = 0.0f;
    gyro_data.frame_count = 0U;
    gyro_data.age_ms = UINT32_MAX;
    gyro_data.valid = false;
    last_raw_angle = 0.0f;
    last_frame = 0U;
    last_frame_tick = BSP_GetTick();
    zero_locked = false;
}

void GyroFusion_Update(void)
{
    GyroData_t raw_data;
    uint32_t now = BSP_GetTick();

    if (Gyro_GetData(&raw_data) && raw_data.frame_count != last_frame) {
        if (!zero_locked) {
            gyro_lock_zero(&raw_data);
        } else {
            float angle_delta = GYRO_DIRECTION_SIGN
                * WrapAngle180(raw_data.angle_deg - last_raw_angle);
            uint32_t dt_ms = now - last_frame_tick;

            /* Some firmware revisions keep the reported angle nearly fixed
             * while still updating angular velocity.  Do not let that make
             * the heading freeze during a line-followed arc. */
            if (AbsFloat(angle_delta) < GYRO_RATE_FALLBACK_MIN_DELTA_DEG
                && AbsFloat(raw_data.dps) >= GYRO_RATE_FALLBACK_MIN_DPS
                && dt_ms <= GYRO_DT_MAX_MS) {
                angle_delta = GYRO_DIRECTION_SIGN * raw_data.dps
                    * ((float)dt_ms * 0.001f);
            }
            gyro_data.heading += angle_delta;
            gyro_data.fused_angle = WrapAngle180(gyro_data.heading);
            gyro_data.angle_deg = GYRO_DIRECTION_SIGN * raw_data.angle_deg;
            gyro_data.dps = GYRO_DIRECTION_SIGN * raw_data.dps;
            last_raw_angle = raw_data.angle_deg;
        }
        last_frame = raw_data.frame_count;
        last_frame_tick = now;
        gyro_data.frame_count = raw_data.frame_count;
        gyro_data.age_ms = 0U;
        gyro_data.valid = true;
    } else if (last_frame == 0U) {
        gyro_data.valid = false;
        gyro_data.age_ms = UINT32_MAX;
    } else {
        gyro_data.age_ms = now - last_frame_tick;
        gyro_data.valid = gyro_data.age_ms <= GYRO_DATA_TIMEOUT_MS;
    }
}

void GyroFusion_Zero(void)
{
    GyroData_t raw;

    /* 启动前小车应保持静止；重新采样当前温度/供电条件下的 Z 轴零偏。 */
    Gyro_CalibrateBias();
    if (Gyro_GetData(&raw)) {
        gyro_lock_zero(&raw);
        last_frame = raw.frame_count;
        last_frame_tick = BSP_GetTick();
        gyro_data.frame_count = raw.frame_count;
        gyro_data.age_ms = 0U;
        gyro_data.valid = true;
    } else {
        zero_locked = false;
        gyro_data.valid = false;
    }
}

float GyroFusion_GetAngle(void)
{
    return gyro_data.fused_angle;
}

float GyroFusion_GetHeading(void)
{
    return gyro_data.heading;
}

float GyroFusion_GetRate(void)
{
    return gyro_data.dps;
}

bool GyroFusion_IsValid(void)
{
    return gyro_data.valid;
}

GyroFusion_DataDef GyroFusion_GetData(void)
{
    return gyro_data;
}
