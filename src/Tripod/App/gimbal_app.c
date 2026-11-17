#include "gimbal_app.h"
#include "main.h"
#include "usart.h"
#include "../Bsp/gimbal_ctrl.h"
#include "../Drv/drv_uart.h"
#include "../Bsp/board_comm.h"
#include "../Bsp/k230_uart.h"
#include "../Bsp/gyro_uart.h"
#include "../Bsp/key.h"
#include "../Bsp/bsp_init.h"
#include "../Bsp/oled_i2c.h"
#include "../Bsp/laser_ctrl.h"
#include "../Lib/comm_protocol.h"
#include <stdio.h>

/* ---- 定时参数 ---- */
#define CTRL_PROCESS_PERIOD_MS   5     /* 控制环周期 5ms */
#define HEARTBEAT_INTERVAL_MS    1000  /* 心跳间隔 1s */
#define OLED_UPDATE_INTERVAL_MS  500   /* OLED刷新间隔 500ms */

/* ---- 跟踪帧计数阈值 ---- */
#define TRACKING_LOCK_FRAMES     5     /* 连续跟踪帧数达到此值进入LOCKED */
#define TRACKING_LOST_FRAMES     10    /* 目标丢失帧数超过此值回到SEARCH */
#define SYNC_DRAW_LOST_FRAMES    30    /* 同步画圆模式下目标丢失阈值 */

/* ---- 模块状态 ---- */
static GimbalApp_StatusTypeDef g_status;

typedef enum {
    AIM_STAGE_IDLE = 0,
    AIM_STAGE_SEARCH_TARGET,
    AIM_STAGE_FIND_LASER,
    AIM_STAGE_CLOSED_LOOP,
    AIM_STAGE_LOCKED,
    AIM_STAGE_STOPPED
} AimStage_TypeDef;

static AimStage_TypeDef g_aim_stage = AIM_STAGE_IDLE;

/* ---- 定时戳 ---- */
static uint32_t g_ctrl_tick       = 0;
static uint32_t g_heartbeat_time = 0;
static uint32_t g_oled_update_time = 0;
static uint32_t g_uart_health_time = 0;

/* ---- 帧计数 ---- */
static uint16_t g_track_frame_cnt  = 0;  /* 连续跟踪帧计数 */
static uint16_t g_lost_frame_cnt   = 0;  /* 连续丢失帧计数 */
static uint32_t g_last_target_frame = 0;
static uint32_t g_last_laser_frame = 0;
static uint32_t g_last_deviation_frame = 0;
static uint32_t g_last_hit_frame = 0;
static bool g_target_was_valid = false;
static bool g_laser_was_valid = false;

/* ---- 按键切换 ---- */
static bool g_aim_toggle = false;
static bool g_pitch_slept = false;  /* 俯仰电机是否已切到低功耗模式 */
static bool g_manual_sleep = false; /* 手动休眠控制标志 */
static bool g_key1_pressed = false; /* KEY1按下提示标志 */
static uint32_t g_key1_show_time = 0; /* KEY1提示显示时间 */
static uint32_t g_task_start_time = 0;

/* ---- 串口调试缓冲区（直接UART收发，不经过Motor框架）---- */
static uint8_t g_dbg_tx_buf[32];
static uint8_t g_dbg_tx_len = 0;
static uint8_t g_dbg_rx_buf[64];
static uint8_t g_dbg_rx_len = 0;

/* ---- 前向声明 ---- */
static void ProcessKeyInput(void);
static void ProcessCarData(void);
static void ProcessK230Data(void);
static void ProcessStateMachine(void);
static void UpdateOLED(void);
static const char *GetAimStageText(void);
static const char *GetAppStateText(void);
static void StartAimTask(void);
static void StopAimTask(void);

/* ============================================================
 *  初始化
 * ============================================================ */
void GimbalApp_Init(void)
{
    /* BSP_Init() 已在 main() 中调用，此处不再重复调用 */
    /* GimbalCtrl/K230/Gyro/Laser已由main()中的BSP_Init()初始化。 */
    BoardComm_Init();
    Key_Init();
    OLED_I2C_Init();

    g_status.state           = GIMBAL_APP_STATE_IDLE;
    g_status.rotation_angle  = GimbalCtrl_GetRotationAngle();
    g_status.pitch_angle     = GimbalCtrl_GetPitchAngle();
    g_status.emergency_stop  = false;

    g_ctrl_tick        = HAL_GetTick();
    g_heartbeat_time   = HAL_GetTick();
    g_oled_update_time = HAL_GetTick();

    g_track_frame_cnt = 0;
    g_lost_frame_cnt  = 0;
    g_last_target_frame = 0;
    g_last_laser_frame = 0;
    g_last_deviation_frame = 0;
    g_last_hit_frame = 0;
    g_target_was_valid = false;
    g_laser_was_valid = false;
    g_aim_toggle      = false;
    g_aim_stage       = AIM_STAGE_IDLE;
    g_task_start_time = 0;

    /* 上电待机，按KEY2后开始搜索/瞄准任务。 */
    GimbalCtrl_SetMode(GIMBAL_MODE_STOP);
    GimbalCtrl_LaserOff();
}

/* ============================================================
 *  主循环 — 在 main() while(1) 中反复调用
 * ============================================================ */
void GimbalApp_MainLoop(void)
{
    /* 1. 按键扫描 */
    Key_Update();
    ProcessKeyInput();

    /* 2. 板间通信 */
    BoardComm_Process();
    ProcessCarData();

    /* 3. K230 视觉数据 */
    K230_UART_Process();
    ProcessK230Data();

    /* 4. 陀螺仪数据 → 云台控制 */
    Gyro_UART_Process();
    {
        Gyro_DataTypeDef gyro = Gyro_UART_GetData();
        GimbalCtrl_UpdateGyroData(gyro.yaw, gyro.yaw_rate, gyro.valid);

        /* 自动休眠逻辑已禁用，改为KEY1手动控制 */
        /* 原逻辑：首次收到有效陀螺仪数据 → 俯仰电机切到低功耗模式（降低EMI） */
        // if (gyro.valid && !g_pitch_slept) {
        //     GimbalCtrl_PitchLowPower();
        //     g_pitch_slept = true;
        // }
    }

    /* 5. 5ms 控制节拍 */
    {
        uint32_t now = HAL_GetTick();
        if (now - g_ctrl_tick >= CTRL_PROCESS_PERIOD_MS) {
            g_ctrl_tick = now;
            GimbalCtrl_Process();
        }
    }

    /* 6. 心跳 1s — 同时发给小车和K230 */
    {
        uint32_t now = HAL_GetTick();
        if (now - g_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
            BoardComm_SendHeartbeat();              /* → 小车 */
            K230_UART_SendCommand(K230_CMD_HEARTBEAT, NULL, 0);  /* → K230 */
            g_heartbeat_time = now;
        }
    }

    /* 7. OLED 刷新 500ms */
    {
        uint32_t now = HAL_GetTick();
        if (now - g_oled_update_time >= OLED_UPDATE_INTERVAL_MS) {
            UpdateOLED();
            g_oled_update_time = now;
        }
    }

    /* 8. UART 健康看门狗 200ms */
    {
        uint32_t now = HAL_GetTick();
        if (now - g_uart_health_time >= 200) {
            DrvUART_HealthCheck(3);  /* 索引3 = UART4(陀螺仪) */
            DrvUART_HealthCheck(0);  /* 索引0 = USART1(K230) */
            g_uart_health_time = now;
        }
    }

    /* 9. 状态机动作 */
    ProcessStateMachine();

    /* 同步角度到状态结构 */
    g_status.rotation_angle = GimbalCtrl_GetRotationAngle();
    g_status.pitch_angle    = GimbalCtrl_GetPitchAngle();
}

/* ============================================================
 *  紧急停止
 * ============================================================ */
void GimbalApp_EmergencyStop(void)
{
    g_status.emergency_stop = true;
    g_status.state = GIMBAL_APP_STATE_STOPPED;
    GimbalCtrl_SetMode(GIMBAL_MODE_STOP);
    GimbalCtrl_LaserOff();
    LaserCtrl_Off();
}

/* ============================================================
 *  获取状态
 * ============================================================ */
GimbalApp_StatusTypeDef GimbalApp_GetStatus(void)
{
    return g_status;
}

/* ============================================================
 *  处理小车数据
 * ============================================================ */
static void ProcessCarData(void)
{
    /* 更新小车连接状态和边沿 */
    GimbalCtrl_SetCarConnected(BoardComm_IsCarConnected());

    if (BoardComm_IsCarConnected()) {
        Comm_CarStatusData car_status = BoardComm_GetCarStatus();
        GimbalCtrl_UpdateCarEdge((uint8_t)car_status.current_edge);
    }

    /* 收到 START_AIMING: 与KEY2一致，启动完整搜索/瞄准任务。 */
    if (BoardComm_IsStartAimingReceived()) {
        StartAimTask();
    }

    /* 收到 STOP_AIMING: 停止 */
    if (BoardComm_IsStopAimingReceived()) {
        StopAimTask();
    }
}

/* ============================================================
 *  处理 K230 视觉数据
 * ============================================================ */
static void ProcessK230Data(void)
{
    /* 靶心数据 */
    K230_TargetData target = K230_UART_GetTargetData();
    if (target.frame_count != g_last_target_frame) {
        g_last_target_frame = target.frame_count;
        GimbalCtrl_UpdateK230Target(target.x, target.y, true);
        g_target_was_valid = true;
        g_lost_frame_cnt = 0;
        if (g_track_frame_cnt < UINT16_MAX) {
            g_track_frame_cnt++;
        }
    } else if (g_target_was_valid && !target.valid) {
        /* 只在有效->超时的边沿通知一次，避免主循环重复做丢失预测。 */
        GimbalCtrl_UpdateK230Target(target.x, target.y, false);
        g_target_was_valid = false;
        g_track_frame_cnt = 0;
        if (g_lost_frame_cnt < UINT16_MAX) {
            g_lost_frame_cnt++;
        }
    }

    /* 激光点数据 */
    K230_LaserData laser = K230_UART_GetLaserData();
    if (laser.frame_count != g_last_laser_frame) {
        g_last_laser_frame = laser.frame_count;
        GimbalCtrl_UpdateK230Laser(laser.x, laser.y, laser.valid);
        g_laser_was_valid = laser.valid;
    } else if (g_laser_was_valid && !laser.valid) {
        GimbalCtrl_UpdateK230Laser(laser.x, laser.y, false);
        g_laser_was_valid = false;
    }

    /* 偏差数据 */
    K230_DeviationData dev = K230_UART_GetDeviationData();
    if (dev.frame_count != g_last_deviation_frame) {
        g_last_deviation_frame = dev.frame_count;
        GimbalCtrl_UpdateK230Deviation(dev.dx, dev.dy, dev.valid);
    }

    /* 命中确认 */
    K230_HitData hit = K230_UART_GetHitData();
    if (hit.valid && hit.frame_count != g_last_hit_frame) {
        g_last_hit_frame = hit.frame_count;
        Comm_GimbalHitData hit_msg;
        hit_msg.hit_distance = hit.distance;
        hit_msg.confidence   = hit.confidence;
        BoardComm_SendHitConfirm(&hit_msg);
    }

}

/* ============================================================
 *  状态机转换逻辑
 * ============================================================ */
static void ProcessStateMachine(void)
{
    switch (g_status.state) {
    /* ---- 空闲 ---- */
    case GIMBAL_APP_STATE_IDLE:
        break;

    /* ---- 搜索 ---- */
    case GIMBAL_APP_STATE_SEARCH:
        if (g_track_frame_cnt >= 1) {
            /* K230 检测到靶心 → 开激光，寻找光斑并进入闭环 */
            g_aim_stage = AIM_STAGE_FIND_LASER;
            g_status.state = GIMBAL_APP_STATE_TRACKING;
            GimbalCtrl_SetMode(GIMBAL_MODE_TRACKING);
            g_track_frame_cnt = 0;
            g_lost_frame_cnt  = 0;
        }
        break;

    /* ---- 跟踪 ---- */
    case GIMBAL_APP_STATE_TRACKING:
        if (g_aim_stage == AIM_STAGE_FIND_LASER && g_laser_was_valid) {
            g_aim_stage = AIM_STAGE_CLOSED_LOOP;
        }
        if (GimbalCtrl_GetHandle()->tracking_locked) {
            g_aim_stage = AIM_STAGE_LOCKED;
            g_status.state = GIMBAL_APP_STATE_LOCKED;
        } else if (g_lost_frame_cnt > TRACKING_LOST_FRAMES) {
            g_aim_stage = AIM_STAGE_SEARCH_TARGET;
            g_status.state = GIMBAL_APP_STATE_SEARCH;
            GimbalCtrl_SetMode(GIMBAL_MODE_SEARCH);
            GimbalCtrl_LaserOff();
            g_track_frame_cnt = 0;
            g_lost_frame_cnt  = 0;
        }
        break;

    /* ---- 锁定 ---- */
    case GIMBAL_APP_STATE_LOCKED:
        /* 保持锁定状态 */
        break;

    /* ---- 同步画圆 ---- */
    case GIMBAL_APP_STATE_SYNC_DRAW:
        if (g_lost_frame_cnt > SYNC_DRAW_LOST_FRAMES) {
            /* 一维云台不做水平扫描，丢失时保持并等待重新识别。 */
            g_status.state = GIMBAL_APP_STATE_TRACKING;
            GimbalCtrl_SetMode(GIMBAL_MODE_TRACKING);
            g_lost_frame_cnt = 0;
        }
        break;

    /* ---- 停止 ---- */
    case GIMBAL_APP_STATE_STOPPED:
        break;

    /* ---- 错误 ---- */
    case GIMBAL_APP_STATE_ERROR:
        GimbalCtrl_SetMode(GIMBAL_MODE_STOP);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  按键处理
 * ============================================================ */
static void ProcessKeyInput(void)
{
    /* KEY1: 手动切换俯仰电机休眠/唤醒（直接UART收发，不经过Motor框架） */
    if (Key_IsPressed(KEY_ID_CALIBRATE)) {
        g_manual_sleep = !g_manual_sleep;
        GimbalCtrl_SetManualSleep(g_manual_sleep);

        g_key1_pressed = true;
        g_key1_show_time = HAL_GetTick();

        /* 构造帧 */
        g_dbg_tx_buf[0] = 0xAA;
        g_dbg_tx_buf[1] = 0x55;
        g_dbg_tx_buf[2] = 0x02;
        g_dbg_tx_buf[3] = 0x19;
        g_dbg_tx_buf[4] = 0x01;
        g_dbg_tx_buf[5] = g_manual_sleep ? 0x00 : 0x01;
        g_dbg_tx_buf[6] = g_dbg_tx_buf[2] + g_dbg_tx_buf[3] + g_dbg_tx_buf[4] + g_dbg_tx_buf[5];
        g_dbg_tx_len = 7;

        /* 阻塞发送 */
        HAL_UART_Transmit(&huart3, g_dbg_tx_buf, 7, 100);

        /* 阻塞接收响应（最多200ms） */
        g_dbg_rx_len = 0;
        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < 200) {
            uint8_t byte;
            if (DrvUART_Available(&huart3) && DrvUART_Read(&huart3, &byte)) {
                if (g_dbg_rx_len < sizeof(g_dbg_rx_buf)) {
                    g_dbg_rx_buf[g_dbg_rx_len++] = byte;
                }
            }
        }

        /* 更新状态（不调用Motor框架的函数，避免冲突） */
        g_pitch_slept = g_manual_sleep;
    }

    /* KEY2: 切换完整任务 搜索->找光斑->闭环->锁定 / 停止 */
    if (Key_IsPressed(KEY_ID_AIM_START)) {
        if (g_aim_toggle) {
            StopAimTask();
        } else {
            StartAimTask();
        }
    }

    /* 同步画圆键 */
    if (Key_IsPressed(KEY_ID_SYNC_DRAW)) {
        g_status.state = GIMBAL_APP_STATE_SYNC_DRAW;
        GimbalCtrl_SetMode(GIMBAL_MODE_SYNC_DRAW);
        K230_UART_SendStartDetection(1);  /* 画圆模式 */
        /* 通知小车开始瞄准，以便协同 */
        BoardComm_SendLaserOn();
        g_track_frame_cnt = 0;
        g_lost_frame_cnt  = 0;
    }
}

/* ============================================================
 *  OLED 显示
 * ============================================================ */
static void UpdateOLED(void)
{
    char buf[48];
    GimbalCtrl_HandleTypeDef *gimbal = GimbalCtrl_GetHandle();
    Gyro_DataTypeDef gyro = Gyro_UART_GetData();
    uint32_t elapsed = 0;

    OLED_I2C_Clear();

    if (g_task_start_time != 0) {
        elapsed = (HAL_GetTick() - g_task_start_time) / 1000U;
    }

    snprintf(buf, sizeof(buf), "TASK %-6s %lus", GetAimStageText(), (unsigned long)elapsed);
    OLED_I2C_ShowString(0, 0, buf);

    snprintf(buf, sizeof(buf), "APP:%-5s L:%c G:%c",
             GetAppStateText(),
             gimbal->laser_on ? 'Y' : 'N',
             gyro.valid ? 'Y' : 'N');
    OLED_I2C_ShowString(0, 12, buf);

    snprintf(buf, sizeof(buf), "T:%c(%3d,%3d) S:%c",
             gimbal->k230_target_valid ? 'Y' : 'N',
             gimbal->k230_target_x,
             gimbal->k230_target_y,
             gimbal->k230_laser_valid ? 'Y' : 'N');
    OLED_I2C_ShowString(0, 24, buf);

    snprintf(buf, sizeof(buf), "Err:%4d,%4d",
             gimbal->k230_target_x - gimbal->k230_laser_x,
             gimbal->k230_target_y - gimbal->k230_laser_y);
    OLED_I2C_ShowString(0, 36, buf);

    snprintf(buf, sizeof(buf), "R:%3.0f P:%3.0f K2:%s",
             g_status.rotation_angle,
             g_status.pitch_angle,
             g_aim_toggle ? "STOP" : "GO");
    OLED_I2C_ShowString(0, 48, buf);

    if (g_key1_pressed && (HAL_GetTick() - g_key1_show_time >= 2000)) {
        g_key1_pressed = false;
    }

    OLED_I2C_Refresh();
}

static void StartAimTask(void)
{
    g_aim_toggle = true;
    g_aim_stage = AIM_STAGE_SEARCH_TARGET;
    g_status.state = GIMBAL_APP_STATE_SEARCH;
    g_task_start_time = HAL_GetTick();

    g_track_frame_cnt = 0;
    g_lost_frame_cnt  = 0;
    g_last_target_frame = 0;
    g_last_laser_frame = 0;
    g_last_deviation_frame = 0;
    g_target_was_valid = false;
    g_laser_was_valid = false;

    K230_UART_SendStartDetection(0);
    GimbalCtrl_LaserOff();
    GimbalCtrl_SetMode(GIMBAL_MODE_SEARCH);
}

static void StopAimTask(void)
{
    g_aim_toggle = false;
    g_aim_stage = AIM_STAGE_STOPPED;
    g_status.state = GIMBAL_APP_STATE_STOPPED;
    g_task_start_time = 0;

    K230_UART_SendStopDetection();
    GimbalCtrl_SetMode(GIMBAL_MODE_STOP);
    GimbalCtrl_LaserOff();
    LaserCtrl_Off();

    g_track_frame_cnt = 0;
    g_lost_frame_cnt  = 0;
}

static const char *GetAimStageText(void)
{
    switch (g_aim_stage) {
        case AIM_STAGE_IDLE:          return "IDLE";
        case AIM_STAGE_SEARCH_TARGET: return "SEARCH";
        case AIM_STAGE_FIND_LASER:    return "LASER";
        case AIM_STAGE_CLOSED_LOOP:   return "AIM";
        case AIM_STAGE_LOCKED:        return "LOCK";
        case AIM_STAGE_STOPPED:       return "STOP";
        default:                      return "UNK";
    }
}

static const char *GetAppStateText(void)
{
    switch (g_status.state) {
        case GIMBAL_APP_STATE_IDLE:      return "IDLE";
        case GIMBAL_APP_STATE_SEARCH:    return "SRCH";
        case GIMBAL_APP_STATE_TRACKING:  return "TRK";
        case GIMBAL_APP_STATE_LOCKED:    return "LOCK";
        case GIMBAL_APP_STATE_SYNC_DRAW: return "SYNC";
        case GIMBAL_APP_STATE_STOPPED:   return "STOP";
        case GIMBAL_APP_STATE_ERROR:     return "ERR";
        default:                         return "UNK";
    }
}
