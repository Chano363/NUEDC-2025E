#include <stdio.h>
#include "oled_app.h"
#include "tracking.h"
#include "Bsp/gyro_uart.h"
#include "gyro_fusion.h"
#include "heading_ctrl.h"
#include "h3_task.h"
#include "Drv/drv_oled.h"
#include "Bsp/bsp_init.h"

#define OLED_FRAME_INTERVAL_MS 200U

static uint32_t g_last_frame_tick;

static const char *const g_state_name[] = {"NOR", "LST", "LSR"};

static void grayscale_to_text(uint8_t digital, char *text)
{
    for (uint8_t i = 0; i < 8; i++) {
        text[i] = (digital & (1U << i)) ? '1' : '0';
    }
    text[8] = '\0';
}

void OLED_App_Init(void)
{
    Drv_OLED_Init();
    Drv_OLED_Clear();
    Drv_OLED_ShowString(0, 0, "System Ready");
    Drv_OLED_Display();
    g_last_frame_tick = BSP_GetTick();
}

void OLED_App_Task(uint32_t now_ms)
{
    char line[20];
    char sensors[9];
    GyroFusion_DataDef gyro = GyroFusion_GetData();
    GyroStatus_t gyro_status = Gyro_GetStatus();
    H3_OutputTypeDef h3 = H3_GetOutput();
    uint8_t state = Tracking_GetState();
    uint8_t h3_active = (h3.running || h3.finished
        || h3.state >= H3_STATE_ERROR_GYRO);

    if (now_ms - g_last_frame_tick < OLED_FRAME_INTERVAL_MS) return;
    g_last_frame_tick = now_ms;
    if (state >= 3U) state = 0U;
    /* H3 reads the grayscale sensors itself in both straight and arc
     * states.  Display that snapshot while H3 is active instead of the
     * background tracking module's possibly stale value. */
    grayscale_to_text(h3_active ? h3.digital_status
                                : Tracking_GetDigitalStatus(), sensors);

    Drv_OLED_Clear();
    snprintf(line, sizeof(line), "%s %s", sensors, g_state_name[state]);
    Drv_OLED_ShowString(0, 0, line);
    snprintf(line, sizeof(line), "H%+5.1f R%+4.0f",
        gyro.heading, gyro.dps);
    Drv_OLED_ShowString(2, 0, line);
    snprintf(line, sizeof(line), "G:%s W%02X A%02X",
        gyro_status.initialized ? "OK" : "ER",
        gyro_status.who_am_i, gyro_status.address);
    Drv_OLED_ShowString(4, 0, line);
    snprintf(line, sizeof(line), "B%+4.1f E%03lu H3:%c",
        gyro_status.bias_dps,
        (unsigned long)(gyro_status.error_count % 1000U),
        HeadingCtrl_ZeroHoldIsActive() ? 'Z' : H3_GetStateCode());
    Drv_OLED_ShowString(6, 0, line);
    Drv_OLED_Display();
}
