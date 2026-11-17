/* ── 主循环调度层 ── */
#include <stdio.h>
#include <stdlib.h>
#include "ti_msp_dl_config.h"
#include "Bsp/bsp_init.h"
#include "Bsp/bluetooth.h"
#include "Bsp/motor_drv.h"
#include "Bsp/encoder.h"
#include "Bsp/grayscale.h"
#include "Bsp/gyro_uart.h"
#include "Bsp/key.h"
#include "App/gyro_fusion.h"
#include "App/heading_ctrl.h"
#include "App/h3_task.h"
#include "App/odometry.h"
#include "App/tracking.h"
#include "App/speed_ctrl.h"
#include "App/oled_app.h"
#include "Drv/drv_uart.h"

#define ENCODER_TEST_DISABLED 0U

/* ══════════════════════════════════════════════════════════════
   蓝牙调参命令解析
   格式: XX=数值D   查询: ??D
   KP=0.5D  KI=0.01D  KD=0.9D  BS=100D  SS=80D  ??D
   ══════════════════════════════════════════════════════════════ */

static void parse_param_command(const Bluetooth_CmdDef *cmd)
{
    const uint8_t *buf = cmd->cmd_buffer;
    uint16_t len = cmd->cmd_length;

    /* 最短命令: ??D (3字节) */
    if (len < 3) return;

    /* 查询命令: ??D */
    if (buf[0] == '?' && buf[1] == '?') {
        char resp[100];
        int n = snprintf(resp, sizeof(resp),
            "KP=%.2f KI=%.3f KD=%.2f BS=%d SS=%d\r\n",
            Tracking_GetKp(), Tracking_GetKi(), Tracking_GetKd(),
            Tracking_GetBaseSpeed(), Tracking_GetSearchSpeed());
        Bluetooth_SendData((const uint8_t *)resp, (uint16_t)n);
        return;
    }

    /* 设置命令: XX=数值D，至少4字节 (KP=D) */
    if (len < 4 || buf[1] != '=') return;

    /* 提取数值字符串: buf[2] ~ buf[len-2]（末尾是D） */
    char val_str[16];
    uint16_t val_len = len - 3; /* 去掉 XX= 和末尾D */
    if (val_len >= sizeof(val_str)) val_len = sizeof(val_str) - 1;
    for (uint16_t i = 0; i < val_len; i++) {
        val_str[i] = (char)buf[2 + i];
    }
    val_str[val_len] = '\0';

    char resp[48];
    int n;

    switch (buf[0]) {
    case 'P': /* KP */
        Tracking_SetKp((float)atof(val_str));
        n = snprintf(resp, sizeof(resp), "KP=%.2f\r\n", Tracking_GetKp());
        Bluetooth_SendData((const uint8_t *)resp, (uint16_t)n);
        break;
    case 'I': /* KI */
        Tracking_SetKi((float)atof(val_str));
        n = snprintf(resp, sizeof(resp), "KI=%.3f\r\n", Tracking_GetKi());
        Bluetooth_SendData((const uint8_t *)resp, (uint16_t)n);
        break;
    case 'D': /* KD — D=x 不会被误判为终止符，因为终止符在末尾 */
        Tracking_SetKd((float)atof(val_str));
        n = snprintf(resp, sizeof(resp), "KD=%.2f\r\n", Tracking_GetKd());
        Bluetooth_SendData((const uint8_t *)resp, (uint16_t)n);
        break;
    case 'B': /* BS=base_speed */
        Tracking_SetBaseSpeed((int16_t)atoi(val_str));
        n = snprintf(resp, sizeof(resp), "BS=%d\r\n", Tracking_GetBaseSpeed());
        Bluetooth_SendData((const uint8_t *)resp, (uint16_t)n);
        break;
    case 'S': /* SS=search_speed */
        Tracking_SetSearchSpeed((int16_t)atoi(val_str));
        n = snprintf(resp, sizeof(resp), "SS=%d\r\n", Tracking_GetSearchSpeed());
        Bluetooth_SendData((const uint8_t *)resp, (uint16_t)n);
        break;
    default:
        Bluetooth_SendString("ERR:unknown param\r\n");
        break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    BSP_Init();
    Key_Init();
    Bluetooth_Init();
    /* Encoder_Init(); -- 暂时停用编码器，开环运行 */
    Grayscale_Init();
    Tracking_Init();
    SpeedCtrl_Init();
    Gyro_Init();
    GyroFusion_Init();
    GyroFusion_Zero();
    Odometry_Init();
    H3_Init();
    OLED_App_Init();

    /* ── 启动提示 ── */
    Bluetooth_SendString("=== Tracking ===\r\n");
    Bluetooth_SendString("KEY1 long-press: start\r\n");

    /* ── 状态变量 ── */
    uint32_t last_report_tick = 0;

    /* ── 主循环 ── */
    while (1)
    {
        uint32_t now_ms = BSP_GetTick();

        GyroFusion_Update();
        Key_Update(now_ms);

        if (Key_GetLongPressEvent(KEY_ID_MODE)) {
            H3_CycleMode();
            Bluetooth_SendString("MODE NEXT\r\n");
        }

        /* 长按KEY1启动循迹 */
        if (Key_GetLongPressEvent(KEY_ID_START)) {
            HeadingCtrl_ZeroHoldStop();
            H3_Start(now_ms);
            Bluetooth_SendString("H3 START\r\n");
        }

        /* 长按 KEY3：启动/停止独立的 0°原地回正测试。 */
        if (Key_GetLongPressEvent(KEY_ID_EMERGENCY)) {
            if (HeadingCtrl_ZeroHoldIsActive()) {
                HeadingCtrl_ZeroHoldStop();
                SpeedCtrl_EmergencyStop();
                Bluetooth_SendString("ZERO HOLD STOP\r\n");
            } else if (GyroFusion_IsValid()) {
                HeadingCtrl_ZeroHoldStart();
                Bluetooth_SendString("ZERO HOLD START\r\n");
            }
        }

        /* ── 事件处理：蓝牙命令 ── */
        if (Bluetooth_HasCommand()) {
            Bluetooth_CmdDef cmd;
            Bluetooth_GetCommand(&cmd);
            Bluetooth_ClearCommand();

            if (cmd.cmd_length >= 3
                && cmd.cmd_buffer[0] == 'N'
                && cmd.cmd_buffer[1] == '='
                && cmd.cmd_buffer[2] >= '1' && cmd.cmd_buffer[2] <= '9') {
                Tracking_Start(cmd.cmd_buffer[2] - '0');
                Bluetooth_SendString("START\r\n");
            }
            else if (cmd.cmd_length == 5
                     && cmd.cmd_buffer[0] == 'S'
                     && cmd.cmd_buffer[1] == 'T'
                     && cmd.cmd_buffer[2] == 'O'
                     && cmd.cmd_buffer[3] == 'P') {
                Tracking_Stop();
                SpeedCtrl_EmergencyStop();
                Bluetooth_SendString("STOPPED\r\n");
            }
            else {
                /* 尝试解析调参命令 (P=/I=/D=/B=/S=/??) */
                parse_param_command(&cmd);
            }
        }

        /* ── 处理UART发送── */
        Drv_UART_ProcessTX();

        /* ── 定时任务：20ms控制周期（由SysTick计数触发）── */
        if (BSP_ControlTick()) {
            H3_OutputTypeDef h3_output;
            Tracking_OutputDef output;

            if (HeadingCtrl_ZeroHoldIsActive()) {
                int16_t left;
                int16_t right;
                float error;
                HeadingCtrl_ZeroHoldUpdate(&left, &right, &error);
                SpeedCtrl_Update(left, right);
            } else {
                H3_Update(now_ms);
                h3_output = H3_GetOutput();
                if (h3_output.running || h3_output.finished
                    || h3_output.state >= H3_STATE_ERROR_GYRO) {
                    SpeedCtrl_Update(h3_output.target_left, h3_output.target_right);
                } else {
                    Tracking_Update();
                    output = Tracking_GetOutput();
                    SpeedCtrl_Update(output.target_left, output.target_right);
                }
            }
            Odometry_Update();
        }

        /* OLED is diagnostic only; keep it independent of the motor tick. */
        OLED_App_Task(now_ms);

        /* ── 定时任务：200ms状态报告 ── */
        uint32_t now = BSP_GetTick();

        if (now - last_report_tick >= 200) {
            last_report_tick = now;
            Tracking_OutputDef output = Tracking_GetOutput();

            uint8_t raw = Tracking_GetDigitalStatus();
            uint8_t state = Tracking_GetState();
            const char *state_name[] = {"NOR", "LST", "LSR"};  /* NORMAL/LOST_STRAIGHT/LOST_SEARCH */

            char buf[128];
            int n = snprintf(buf, sizeof(buf),
                "raw=0x%02X dev=%d tgt=%d/%d ST=%s %s KP=%.1f KI=%.2f KD=%.1f BS=%d\r\n",
                raw,
                Tracking_GetDeviation(),
                output.target_left, output.target_right,
                state_name[state],
                output.running ? "RUN" : "WAIT",
                Tracking_GetKp(), Tracking_GetKi(), Tracking_GetKd(),
                Tracking_GetBaseSpeed());

            Bluetooth_SendData((const uint8_t *)buf, (uint16_t)n);
        }
    }
}
