#include "gimbal_ctrl.h"
#include "stepper_motor.h"
#include "laser_ctrl.h"
#include "gyro_uart.h"
#include "k230_uart.h"
#include "../Lib/pid.h"
#include "../Lib/math_utils.h"
#include "../Drv/drv_uart.h"
#include "main.h"
#include "usart.h"
#include <math.h>

/* ======================== 常量定义 ======================== */

#define GIMBAL_ROTATION_NEUTRAL  135.0f
#define GIMBAL_PITCH_NEUTRAL     90.0f
#define GIMBAL_ROTATION_MIN      0.0f
#define GIMBAL_ROTATION_MAX      270.0f
#define GIMBAL_PITCH_MIN         0.0f
#define GIMBAL_PITCH_MAX         180.0f

#define K230_IMAGE_CENTER_X      160
#define K230_IMAGE_CENTER_Y      120

/* 内环PID参数（陀螺仪稳定，200Hz） */
#define INNER_ROT_KP             5.0f
#define INNER_ROT_KI             0.5f
#define INNER_ROT_KD             0.8f
#define INNER_PITCH_KP           3.0f
#define INNER_PITCH_KI           0.3f
#define INNER_PITCH_KD           0.5f
#define INNER_INTEGRAL_LIMIT     10.0f   // 度

/* 外环PID参数（视觉跟踪，~30Hz） */
#define OUTER_ROT_KP             1.5f
#define OUTER_ROT_KI             0.1f
#define OUTER_ROT_KD             0.3f
#define OUTER_PITCH_KP           1.0f
#define OUTER_PITCH_KI           0.05f
#define OUTER_PITCH_KD           0.2f
#define OUTER_INTEGRAL_LIMIT     50.0f   // 度
#define OUTER_OUTPUT_LIMIT       10.0f   // 度

/* 误差死区 */
#define PID_ERROR_DEADZONE       5.0f

/* 第二问定点瞄准：用“靶心 - 激光点”的像素误差做二维闭环。 */
#define AIM_ROT_KP               0.04f  /* 度/像素 */
#define AIM_PITCH_KP             0.04f  /* 度/像素 */
#define AIM_ROT_MAX_STEP         2.0f   /* 单帧最大水平修正角度 */
#define AIM_PITCH_MAX_STEP       2.0f   /* 单帧最大俯仰修正角度 */
#define AIM_ROT_DIRECTION        1.0f   /* 若实机水平方向相反，改为-1.0f */
#define AIM_PITCH_DIRECTION      1.0f   /* 若实机俯仰方向相反，改为-1.0f */
#define AIM_STABLE_THRESHOLD     5.0f   /* 像素，激光点距靶心足够近 */

/* 锁定判定 */
#define LOCK_THRESHOLD           5       // 像素
#define LOCK_FRAME_COUNT         5       // 连续帧数

/* 突变前馈参数 */
#define FEEDFORWARD_YAW_THRESHOLD  4.0f  // 度，突变判定阈值
#define FEEDFORWARD_YAW_MAX        100.0f // 度，突变上限（超过则为异常数据）
#define FEEDFORWARD_GAIN           0.8f   // 前馈增益

/* 角速度前馈增益 */
#define VELOCITY_FF_ROTATION       0.8f
#define VELOCITY_FF_PITCH          0.6f

/* 搜索参数 */
#define SEARCH_SPEED               30.0f  // 度/秒
#define TARGET_LOST_MAX            10     // 帧数，丢失后切换搜索
#define SEARCH_ROT_MIN             30.0f
#define SEARCH_ROT_MAX             240.0f
#define SEARCH_PITCH_LEVEL_0       90.0f
#define SEARCH_PITCH_LEVEL_1       80.0f
#define SEARCH_PITCH_LEVEL_2       100.0f
#define SEARCH_EDGE_HOLD_TICKS     40U    /* 约200ms */

/* 找到靶面但还未看到激光点时，先让靶心进入画面中心。 */
#define TARGET_CENTER_KP           0.03f
#define TARGET_CENTER_MAX_STEP     1.5f

/* 同步描圆参数 */
#define SYNC_DRAW_EDGE_TIME        5.0f   // 每边秒数（20秒一圈 / 4边）
#define SYNC_DRAW_RADIUS_PX        45.0f  // 像素，6cm半径在典型距离的映射

/* ======================== 静态变量 ======================== */

static GimbalCtrl_HandleTypeDef g_gimbal;
static bool     g_pitch_lowpower = false;  /* 俯仰电机是否处于低功耗 */
static uint32_t g_pitch_sleep_tick = 0;    /* 进入低功耗的时间戳 */
static bool     g_manual_sleep = false;    /* 手动休眠标志 */
#define GIMBAL_PITCH_SLEEP_MIN_MS  10000   /* 最低休眠时间(ms)，10秒 */
static uint8_t  g_search_pitch_level = 0;
static uint16_t g_search_edge_hold = 0;

/* 电机句柄 */
static Motor_HandleTypeDef g_rotation_motor;  // 水平，hlpuart1，地址0x01
static Motor_HandleTypeDef g_pitch_motor;     // 俯仰，huart3，地址0x01

/* 内环PID（陀螺仪稳定） */
static PID_HandleTypeDef g_pid_inner_rotation;  // Kp=5.0, Ki=0.5, Kd=0.8
static PID_HandleTypeDef g_pid_inner_pitch;     // Kp=3.0, Ki=0.3, Kd=0.5

/* 外环PID（视觉跟踪） */
static PID_HandleTypeDef g_pid_outer_rotation;  // Kp=1.5, Ki=0.1, Kd=0.3
static PID_HandleTypeDef g_pid_outer_pitch;     // Kp=1.0, Ki=0.05, Kd=0.2

/* 外环时间戳（用于动态dt） */
static uint32_t g_outer_last_tick = 0;
static bool g_outer_first_frame = true;

/* 惯性预测上帧数据 */
static int16_t g_prev_target_x = K230_IMAGE_CENTER_X;
static int16_t g_prev_target_y = K230_IMAGE_CENTER_Y;
static uint32_t g_prev_target_tick = 0;
static float g_prev_target_dt = 0.033f;  // 默认~30Hz

/* ======================== 前向声明 ======================== */

static void GimbalCtrl_ProcessSearch(void);
static void GimbalCtrl_ProcessTracking(void);
static void GimbalCtrl_ProcessLocked(void);
static void GimbalCtrl_ProcessSyncDraw(void);
static void GimbalCtrl_ApplyMotors(float rot, float pitch);
static void GimbalCtrl_ApplyAimError(int16_t dx, int16_t dy);
static bool GimbalCtrl_HasAimFeedback(void);
static void GimbalCtrl_TryAimFromVision(void);
static void GimbalCtrl_ApplyTargetCentering(void);
static float GimbalCtrl_GetSearchPitch(uint8_t level);

/* ======================== 初始化 ======================== */

void GimbalCtrl_Init(void)
{
    /* 电机初始化 */
    Motor_Init(&g_rotation_motor, &hlpuart1, MOTOR_ADDR_HORIZONTAL);
    Motor_Init(&g_pitch_motor, &huart3, MOTOR_ADDR_VERTICAL);


    /* 内环PID初始化 */
    g_pid_inner_rotation = PID_Create(INNER_ROT_KP, INNER_ROT_KI, INNER_ROT_KD, PID_MODE_POSITION);
    g_pid_inner_pitch    = PID_Create(INNER_PITCH_KP, INNER_PITCH_KI, INNER_PITCH_KD, PID_MODE_POSITION);
    PID_SetOutputLimit(&g_pid_inner_rotation, 30.0f);
    PID_SetOutputLimit(&g_pid_inner_pitch, 20.0f);
    PID_SetIntegralLimit(&g_pid_inner_rotation, INNER_INTEGRAL_LIMIT);
    PID_SetIntegralLimit(&g_pid_inner_pitch, INNER_INTEGRAL_LIMIT);

    /* 外环PID初始化 */
    g_pid_outer_rotation = PID_Create(OUTER_ROT_KP, OUTER_ROT_KI, OUTER_ROT_KD, PID_MODE_POSITION);
    g_pid_outer_pitch    = PID_Create(OUTER_PITCH_KP, OUTER_PITCH_KI, OUTER_PITCH_KD, PID_MODE_POSITION);
    PID_SetOutputLimit(&g_pid_outer_rotation, OUTER_OUTPUT_LIMIT);
    PID_SetOutputLimit(&g_pid_outer_pitch, OUTER_OUTPUT_LIMIT);
    PID_SetIntegralLimit(&g_pid_outer_rotation, OUTER_INTEGRAL_LIMIT);
    PID_SetIntegralLimit(&g_pid_outer_pitch, OUTER_INTEGRAL_LIMIT);

    /* 云台状态初始化 */
    g_gimbal.target_rotation = GIMBAL_ROTATION_NEUTRAL;
    g_gimbal.target_pitch    = GIMBAL_PITCH_NEUTRAL;
    g_gimbal.current_rotation = GIMBAL_ROTATION_NEUTRAL;
    g_gimbal.current_pitch    = GIMBAL_PITCH_NEUTRAL;
    g_gimbal.mode = GIMBAL_MODE_STOP;
    g_gimbal.tracking_locked = false;
    g_gimbal.laser_on = false;

    g_gimbal.gyro_yaw = 0.0f;
    g_gimbal.gyro_yaw_rate = 0.0f;
    g_gimbal.last_gyro_yaw = 0.0f;
    g_gimbal.gyro_valid = false;

    g_gimbal.k230_target_x = K230_IMAGE_CENTER_X;
    g_gimbal.k230_target_y = K230_IMAGE_CENTER_Y;
    g_gimbal.k230_target_valid = false;
    g_gimbal.k230_laser_x = K230_IMAGE_CENTER_X;
    g_gimbal.k230_laser_y = K230_IMAGE_CENTER_Y;
    g_gimbal.k230_laser_valid = false;
    g_gimbal.k230_deviation_dx = 0;
    g_gimbal.k230_deviation_dy = 0;
    g_gimbal.k230_deviation_valid = false;

    g_gimbal.car_edge = 0;
    g_gimbal.car_connected = false;

    g_gimbal.search_dir = SEARCH_DIR_RIGHT;
    g_gimbal.search_speed = SEARCH_SPEED;
    g_gimbal.search_current_angle = GIMBAL_ROTATION_NEUTRAL;

    g_gimbal.target_vx = 0.0f;
    g_gimbal.target_vy = 0.0f;
    g_gimbal.target_lost_count = 0;
    g_gimbal.predicted_x = K230_IMAGE_CENTER_X;
    g_gimbal.predicted_y = K230_IMAGE_CENTER_Y;

    g_gimbal.sync_phase = 0.0f;
    g_gimbal.sync_edge_start_time = 0;

    g_gimbal.feedforward_accumulated = 0.0f;
    g_gimbal.lock_count = 0;

    /* 激光初始化 */
    LaserCtrl_Init();
    LaserCtrl_Off();

    g_gimbal.current_rotation = GIMBAL_ROTATION_NEUTRAL;
    g_gimbal.target_rotation = GIMBAL_ROTATION_NEUTRAL;
    g_gimbal.current_pitch = GIMBAL_PITCH_NEUTRAL;
    g_gimbal.target_pitch = GIMBAL_PITCH_NEUTRAL;
    g_rotation_motor.current_angle = (int32_t)GIMBAL_ROTATION_NEUTRAL;
    g_pitch_motor.current_angle = (int32_t)GIMBAL_PITCH_NEUTRAL;
    g_search_pitch_level = 0;
    g_search_edge_hold = 0;
}

/* ======================== 主处理函数（每5ms调用） ======================== */

void GimbalCtrl_Process(void)
{
    /* ---- 1. 更新陀螺仪数据 ---- */
    Gyro_DataTypeDef gyro = Gyro_UART_GetData();
    GimbalCtrl_UpdateGyroData(gyro.yaw, gyro.yaw_rate, gyro.valid);

    if (g_gimbal.mode == GIMBAL_MODE_STOP) {
        return;
    }

    /*
     * 跟踪/锁定阶段由新到达的K230帧直接修正角度，避免在5ms节拍里
     * 重复发送相同角度命令。
     */
    if (g_gimbal.mode == GIMBAL_MODE_TRACKING) {
        GimbalCtrl_ProcessTracking();
        return;
    }
    if (g_gimbal.mode == GIMBAL_MODE_LOCKED) {
        GimbalCtrl_ProcessLocked();
        return;
    }

    /* ---- 2. 航向突变前馈（盲转补偿） ---- */
    if (g_gimbal.gyro_valid) {
        float delta_yaw = g_gimbal.gyro_yaw - g_gimbal.last_gyro_yaw;
        /* 处理角度回绕 */
        if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
        if (delta_yaw < -180.0f) delta_yaw += 360.0f;

        if (AbsFloat(delta_yaw) > FEEDFORWARD_YAW_THRESHOLD &&
            AbsFloat(delta_yaw) < FEEDFORWARD_YAW_MAX) {
            /* 突变检测到——一次性前馈补偿，反方向旋转电机保持K230视野 */
            float comp_angle = -delta_yaw * FEEDFORWARD_GAIN;
            g_gimbal.target_rotation += comp_angle;
            g_gimbal.feedforward_accumulated += comp_angle;
            /* 立即发送给电机 */
            GimbalCtrl_SetRotationAngle(g_gimbal.target_rotation);
        }
        g_gimbal.last_gyro_yaw = g_gimbal.gyro_yaw;
    }

    /* ---- 3. 内环PID（陀螺仪稳定，200Hz） ---- */
    {
        float inner_dt = 0.005f;  /* 5ms固定周期 */
        float rot_error = g_gimbal.target_rotation - g_gimbal.current_rotation;
        float pitch_error = g_gimbal.target_pitch - g_gimbal.current_pitch;

        /* 死区处理 */
        if (AbsFloat(rot_error) < 0.1f) rot_error = 0.0f;
        if (AbsFloat(pitch_error) < 0.1f) pitch_error = 0.0f;

        /* PID计算：传入负误差，使setpoint=0时输出与误差同号 */
        float inner_rot_output = PID_ComputeDt(&g_pid_inner_rotation, -rot_error, inner_dt);
        float inner_pitch_output = PID_ComputeDt(&g_pid_inner_pitch, -pitch_error, inner_dt);

        /* 角速度前馈：补偿基座连续旋转 */
        if (g_gimbal.gyro_valid) {
            inner_rot_output += VELOCITY_FF_ROTATION * g_gimbal.gyro_yaw_rate * inner_dt;
        }

        /* 计算新的电机角度 */
        float new_rot = g_gimbal.current_rotation + inner_rot_output;
        float new_pitch = g_gimbal.current_pitch + inner_pitch_output;

        /* 限幅 */
        new_rot = LimitValue(new_rot, GIMBAL_ROTATION_MIN, GIMBAL_ROTATION_MAX);
        new_pitch = LimitValue(new_pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

        /* 应用到电机 */
        GimbalCtrl_ApplyMotors(new_rot, new_pitch);
    }

    /* ---- 4. 外环（视觉PID，在UpdateK230Target中处理） ---- */

    /* ---- 5. 模式逻辑 ---- */
    switch (g_gimbal.mode) {
        case GIMBAL_MODE_SEARCH:
            GimbalCtrl_ProcessSearch();
            break;
        case GIMBAL_MODE_TRACKING:
            GimbalCtrl_ProcessTracking();
            break;
        case GIMBAL_MODE_LOCKED:
            GimbalCtrl_ProcessLocked();
            break;
        case GIMBAL_MODE_SYNC_DRAW:
            GimbalCtrl_ProcessSyncDraw();
            break;
        default:
            break;
    }
}

/* ======================== 搜索模式 ======================== */

static void GimbalCtrl_ProcessSearch(void)
{
    /* 检测到目标：切换到跟踪模式 */
    if (g_gimbal.k230_target_valid) {
        g_gimbal.target_lost_count = 0;
        g_gimbal.lock_count = 0;
        GimbalCtrl_SetMode(GIMBAL_MODE_TRACKING);
        return;
    }

    if (g_search_edge_hold > 0) {
        g_search_edge_hold--;
        return;
    }

    /* 水平往复 + 俯仰分层扫描 */
    float dt = 0.005f;  /* 5ms周期 */
    float step = g_gimbal.search_speed * dt;

    if (g_gimbal.search_dir == SEARCH_DIR_RIGHT) {
        g_gimbal.search_current_angle += step;
        if (g_gimbal.search_current_angle >= SEARCH_ROT_MAX) {
            g_gimbal.search_current_angle = SEARCH_ROT_MAX;
            g_gimbal.search_dir = SEARCH_DIR_LEFT;
            g_search_pitch_level = (uint8_t)((g_search_pitch_level + 1U) % 3U);
            g_gimbal.target_pitch = GimbalCtrl_GetSearchPitch(g_search_pitch_level);
            g_search_edge_hold = SEARCH_EDGE_HOLD_TICKS;
        }
    } else {
        g_gimbal.search_current_angle -= step;
        if (g_gimbal.search_current_angle <= SEARCH_ROT_MIN) {
            g_gimbal.search_current_angle = SEARCH_ROT_MIN;
            g_gimbal.search_dir = SEARCH_DIR_RIGHT;
            g_search_pitch_level = (uint8_t)((g_search_pitch_level + 1U) % 3U);
            g_gimbal.target_pitch = GimbalCtrl_GetSearchPitch(g_search_pitch_level);
            g_search_edge_hold = SEARCH_EDGE_HOLD_TICKS;
        }
    }

    g_gimbal.target_rotation = g_gimbal.search_current_angle;
}

/* ======================== 跟踪模式 ======================== */

static void GimbalCtrl_ProcessTracking(void)
{
    /* 目标或光斑丢失时保持当前位置，等待K230重新识别。 */
    if (!GimbalCtrl_HasAimFeedback()) {
        g_gimbal.target_lost_count++;
        GimbalCtrl_ApplyTargetCentering();
        return;
    }

    /* 视觉反馈有效，重置丢失计数。实际修正在新视觉帧到达时完成。 */
    g_gimbal.target_lost_count = 0;
}

/* ======================== 锁定模式 ======================== */

static void GimbalCtrl_ProcessLocked(void)
{
    /* 激光保持开启 */
    if (!g_gimbal.laser_on) {
        GimbalCtrl_LaserOn();
    }

    /* 监测激光点相对靶心的偏差，偏离过大则继续跟踪修正。 */
    if (GimbalCtrl_HasAimFeedback()) {
        int16_t error_x = g_gimbal.k230_target_x - g_gimbal.k230_laser_x;
        int16_t error_y = g_gimbal.k230_target_y - g_gimbal.k230_laser_y;

        if (AbsFloat((float)error_x) > LOCK_THRESHOLD * 3 ||
            AbsFloat((float)error_y) > LOCK_THRESHOLD * 3) {
            g_gimbal.lock_count = 0;
            GimbalCtrl_SetMode(GIMBAL_MODE_TRACKING);
            return;
        }
    } else {
        /* 目标丢失，降级回搜索 */
        g_gimbal.target_lost_count++;
        if (g_gimbal.target_lost_count > TARGET_LOST_MAX) {
            g_gimbal.lock_count = 0;
            GimbalCtrl_SetMode(GIMBAL_MODE_TRACKING);
            return;
        }
    }
}

/* ======================== 同步描圆模式 ======================== */

static void GimbalCtrl_ProcessSyncDraw(void)
{
    if (!g_gimbal.car_connected) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt_sec = (float)(now - g_gimbal.sync_edge_start_time) / 1000.0f;

    /* 根据小车当前边号和时间计算相位角
     * 每条边对应90°，一圈4条边=360°
     * 假设每边5秒（20秒一圈） */
    float edge_phase = (float)g_gimbal.car_edge * 90.0f;
    float time_phase = (dt_sec / SYNC_DRAW_EDGE_TIME) * 90.0f;
    float phase = edge_phase + time_phase;

    /* 限制相位在0~360 */
    while (phase >= 360.0f) phase -= 360.0f;
    while (phase < 0.0f) phase += 360.0f;
    g_gimbal.sync_phase = phase;

    /* 生成圆上目标点（在图像坐标系中）
     * 圆心=目标中心(160,120)，半径=SYNC_DRAW_RADIUS_PX */
    float rad = phase * 0.01745329f;  /* deg to rad */
    float target_px_x = (float)K230_IMAGE_CENTER_X + SYNC_DRAW_RADIUS_PX * cosf(rad);
    float target_px_y = (float)K230_IMAGE_CENTER_Y + SYNC_DRAW_RADIUS_PX * sinf(rad);

    /* 使用K230偏差数据辅助修正 */
    if (g_gimbal.k230_deviation_valid) {
        target_px_x += (float)g_gimbal.k230_deviation_dx * 0.3f;
        target_px_y += (float)g_gimbal.k230_deviation_dy * 0.3f;
    }

    /* 像素误差 → 外环PID角度修正 */
    float error_x = (float)K230_IMAGE_CENTER_X - target_px_x;
    float error_y = (float)K230_IMAGE_CENTER_Y - target_px_y;

    float dt = 0.005f;
    float outer_rot_output = PID_ComputeDt(&g_pid_outer_rotation, error_x, dt);
    float outer_pitch_output = PID_ComputeDt(&g_pid_outer_pitch, -error_y, dt);

    outer_rot_output = LimitValue(outer_rot_output, -OUTER_OUTPUT_LIMIT, OUTER_OUTPUT_LIMIT);
    outer_pitch_output = LimitValue(outer_pitch_output, -OUTER_OUTPUT_LIMIT, OUTER_OUTPUT_LIMIT);

    g_gimbal.target_rotation += outer_rot_output;
    g_gimbal.target_pitch += outer_pitch_output;

    g_gimbal.target_rotation = LimitValue(g_gimbal.target_rotation,
                                           GIMBAL_ROTATION_MIN, GIMBAL_ROTATION_MAX);
    g_gimbal.target_pitch = LimitValue(g_gimbal.target_pitch,
                                        GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
}

/* ======================== 数据输入接口 ======================== */

void GimbalCtrl_UpdateGyroData(float yaw, float yaw_rate, bool valid)
{
    g_gimbal.gyro_yaw = yaw;
    g_gimbal.gyro_yaw_rate = yaw_rate;
    g_gimbal.gyro_valid = valid;
}

void GimbalCtrl_UpdateK230Target(int16_t x, int16_t y, bool valid)
{
    uint32_t now = HAL_GetTick();

    if (valid) {
        /* 计算目标速度（惯性预测用） */
        if (g_gimbal.k230_target_valid) {
            float dt = (float)(now - g_prev_target_tick) / 1000.0f;
            if (dt > 0.001f) {
                g_prev_target_dt = dt;
                g_gimbal.target_vx = (float)(x - g_prev_target_x) / dt;
                g_gimbal.target_vy = (float)(y - g_prev_target_y) / dt;
            }
        }
        g_prev_target_x = x;
        g_prev_target_y = y;
        g_prev_target_tick = now;

        /* 更新当前目标坐标 */
        g_gimbal.k230_target_x = x;
        g_gimbal.k230_target_y = y;
        g_gimbal.k230_target_valid = true;
        g_gimbal.target_lost_count = 0;

        /* 更新预测坐标 */
        g_gimbal.predicted_x = x;
        g_gimbal.predicted_y = y;

        if (g_gimbal.mode == GIMBAL_MODE_TRACKING || g_gimbal.mode == GIMBAL_MODE_LOCKED) {
            GimbalCtrl_TryAimFromVision();
        }
    } else {
        /* 目标丢失时保持当前位置，禁止旧速度预测造成持续漂移。 */
        g_gimbal.k230_target_valid = false;
        if (g_gimbal.target_lost_count < UINT8_MAX) {
            g_gimbal.target_lost_count++;
        }
        g_gimbal.target_vx = 0.0f;
        g_gimbal.target_vy = 0.0f;
        g_gimbal.lock_count = 0;
        g_gimbal.tracking_locked = false;
        PID_Reset(&g_pid_outer_pitch);
    }
}

void GimbalCtrl_UpdateK230Laser(int16_t x, int16_t y, bool valid)
{
    g_gimbal.k230_laser_x = x;
    g_gimbal.k230_laser_y = y;
    g_gimbal.k230_laser_valid = valid;

    if (g_gimbal.mode == GIMBAL_MODE_TRACKING || g_gimbal.mode == GIMBAL_MODE_LOCKED) {
        GimbalCtrl_TryAimFromVision();
    }
}

void GimbalCtrl_UpdateK230Deviation(int16_t dx, int16_t dy, bool valid)
{
    g_gimbal.k230_deviation_dx = dx;
    g_gimbal.k230_deviation_dy = dy;
    g_gimbal.k230_deviation_valid = valid;

    if (valid && (g_gimbal.mode == GIMBAL_MODE_TRACKING || g_gimbal.mode == GIMBAL_MODE_LOCKED)) {
        GimbalCtrl_ApplyAimError(dx, dy);
    }
}

void GimbalCtrl_UpdateCarEdge(uint8_t edge)
{
    if (edge != g_gimbal.car_edge) {
        g_gimbal.car_edge = edge;
        g_gimbal.sync_edge_start_time = HAL_GetTick();
    }
}

void GimbalCtrl_SetCarConnected(bool connected)
{
    g_gimbal.car_connected = connected;
}

/* ======================== 模式控制 ======================== */

void GimbalCtrl_SetMode(Gimbal_ModeTypeDef mode)
{
    Gimbal_ModeTypeDef old_mode = g_gimbal.mode;
    g_gimbal.mode = mode;

    switch (mode) {
        case GIMBAL_MODE_STOP:
            GimbalCtrl_Stop();
            break;

        case GIMBAL_MODE_SEARCH:
            /* 进入搜索：从当前角度开始扫描 */
            g_gimbal.search_current_angle = g_gimbal.current_rotation;
            g_search_pitch_level = 0;
            g_search_edge_hold = 0;
            g_gimbal.target_pitch = GimbalCtrl_GetSearchPitch(g_search_pitch_level);
            GimbalCtrl_LaserOff();
            if (g_gimbal.current_rotation >= SEARCH_ROT_MAX - 5.0f) {
                g_gimbal.search_dir = SEARCH_DIR_LEFT;
            } else if (g_gimbal.current_rotation <= SEARCH_ROT_MIN + 5.0f) {
                g_gimbal.search_dir = SEARCH_DIR_RIGHT;
            }
            g_gimbal.target_lost_count = 0;
            g_gimbal.lock_count = 0;
            /* 重置外环PID，避免历史积分影响 */
            PID_Reset(&g_pid_outer_rotation);
            PID_Reset(&g_pid_outer_pitch);
            g_outer_first_frame = true;
            break;

        case GIMBAL_MODE_TRACKING:
            g_gimbal.target_lost_count = 0;
            g_gimbal.lock_count = 0;
            g_outer_first_frame = true;
            GimbalCtrl_LaserOn();
            break;

        case GIMBAL_MODE_LOCKED:
            GimbalCtrl_LaserOn();
            break;

        case GIMBAL_MODE_SYNC_DRAW:
            g_gimbal.sync_edge_start_time = HAL_GetTick();
            g_gimbal.sync_phase = (float)g_gimbal.car_edge * 90.0f;
            PID_Reset(&g_pid_outer_rotation);
            PID_Reset(&g_pid_outer_pitch);
            break;

        case GIMBAL_MODE_CALIBRATION:
            GimbalCtrl_ResetToNeutral();
            break;

        default:
            break;
    }

    /* 离开锁定模式时关闭激光 */
    if (old_mode == GIMBAL_MODE_LOCKED && mode != GIMBAL_MODE_LOCKED) {
        if (mode == GIMBAL_MODE_STOP) {
            GimbalCtrl_LaserOff();
        }
    }
}

/* ======================== 第二问二维视觉闭环 ======================== */

static bool GimbalCtrl_HasAimFeedback(void)
{
    return g_gimbal.k230_target_valid && g_gimbal.k230_laser_valid;
}

static float GimbalCtrl_GetSearchPitch(uint8_t level)
{
    switch (level) {
        case 1:  return SEARCH_PITCH_LEVEL_1;
        case 2:  return SEARCH_PITCH_LEVEL_2;
        default: return SEARCH_PITCH_LEVEL_0;
    }
}

static void GimbalCtrl_TryAimFromVision(void)
{
    if (!GimbalCtrl_HasAimFeedback()) {
        return;
    }

    GimbalCtrl_ApplyAimError(g_gimbal.k230_target_x - g_gimbal.k230_laser_x,
                             g_gimbal.k230_target_y - g_gimbal.k230_laser_y);
}

static void GimbalCtrl_ApplyAimError(int16_t dx, int16_t dy)
{
    float error_x = (float)dx;
    float error_y = (float)dy;
    float rot_step = 0.0f;
    float pitch_step = 0.0f;

    g_outer_last_tick = HAL_GetTick();
    g_outer_first_frame = false;
    g_gimbal.target_lost_count = 0;

    if (AbsFloat(error_x) >= PID_ERROR_DEADZONE) {
        rot_step = AIM_ROT_DIRECTION * AIM_ROT_KP * error_x;
        rot_step = LimitValue(rot_step, -AIM_ROT_MAX_STEP, AIM_ROT_MAX_STEP);
    }

    if (AbsFloat(error_y) >= PID_ERROR_DEADZONE) {
        pitch_step = AIM_PITCH_DIRECTION * AIM_PITCH_KP * error_y;
        pitch_step = LimitValue(pitch_step, -AIM_PITCH_MAX_STEP, AIM_PITCH_MAX_STEP);
    }

    if (rot_step != 0.0f || pitch_step != 0.0f) {
        g_gimbal.lock_count = 0;
        g_gimbal.tracking_locked = false;

        g_gimbal.target_rotation += rot_step;
        g_gimbal.target_pitch += pitch_step;
        g_gimbal.target_rotation = LimitValue(g_gimbal.target_rotation,
                                               GIMBAL_ROTATION_MIN, GIMBAL_ROTATION_MAX);
        g_gimbal.target_pitch = LimitValue(g_gimbal.target_pitch,
                                            GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

        GimbalCtrl_SetRotationAngle(g_gimbal.target_rotation);
        GimbalCtrl_SetPitchAngle(g_gimbal.target_pitch);
        return;
    }

    if (AbsFloat(error_x) <= AIM_STABLE_THRESHOLD &&
        AbsFloat(error_y) <= AIM_STABLE_THRESHOLD &&
        g_gimbal.lock_count < UINT8_MAX) {
        g_gimbal.lock_count++;
        if (g_gimbal.lock_count >= LOCK_FRAME_COUNT) {
            g_gimbal.tracking_locked = true;
            GimbalCtrl_SetMode(GIMBAL_MODE_LOCKED);
        }
    }
}

static void GimbalCtrl_ApplyTargetCentering(void)
{
    if (!g_gimbal.k230_target_valid) {
        return;
    }

    float error_x = (float)(K230_IMAGE_CENTER_X - g_gimbal.k230_target_x);
    float error_y = (float)(K230_IMAGE_CENTER_Y - g_gimbal.k230_target_y);
    float rot_step = 0.0f;
    float pitch_step = 0.0f;

    if (AbsFloat(error_x) >= PID_ERROR_DEADZONE) {
        rot_step = AIM_ROT_DIRECTION * TARGET_CENTER_KP * error_x;
        rot_step = LimitValue(rot_step, -TARGET_CENTER_MAX_STEP, TARGET_CENTER_MAX_STEP);
    }

    if (AbsFloat(error_y) >= PID_ERROR_DEADZONE) {
        pitch_step = AIM_PITCH_DIRECTION * TARGET_CENTER_KP * error_y;
        pitch_step = LimitValue(pitch_step, -TARGET_CENTER_MAX_STEP, TARGET_CENTER_MAX_STEP);
    }

    if (rot_step != 0.0f || pitch_step != 0.0f) {
        g_gimbal.target_rotation += rot_step;
        g_gimbal.target_pitch += pitch_step;
        g_gimbal.target_rotation = LimitValue(g_gimbal.target_rotation,
                                               GIMBAL_ROTATION_MIN, GIMBAL_ROTATION_MAX);
        g_gimbal.target_pitch = LimitValue(g_gimbal.target_pitch,
                                            GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

        GimbalCtrl_SetRotationAngle(g_gimbal.target_rotation);
        GimbalCtrl_SetPitchAngle(g_gimbal.target_pitch);
    }
}

void GimbalCtrl_Stop(void)
{
    /* 电机保持当前位置（默认使能），只清PID和激光 */
    PID_Reset(&g_pid_inner_rotation);
    PID_Reset(&g_pid_inner_pitch);
    PID_Reset(&g_pid_outer_rotation);
    PID_Reset(&g_pid_outer_pitch);
    GimbalCtrl_LaserOff();
    g_gimbal.tracking_locked = false;
}

void GimbalCtrl_ResetToNeutral(void)
{
    PID_Reset(&g_pid_inner_rotation);
    PID_Reset(&g_pid_inner_pitch);
    PID_Reset(&g_pid_outer_rotation);
    PID_Reset(&g_pid_outer_pitch);

    g_gimbal.target_rotation = GIMBAL_ROTATION_NEUTRAL;
    g_gimbal.target_pitch = GIMBAL_PITCH_NEUTRAL;
    g_gimbal.current_rotation = GIMBAL_ROTATION_NEUTRAL;
    g_gimbal.current_pitch = GIMBAL_PITCH_NEUTRAL;
    g_gimbal.tracking_locked = false;
    g_gimbal.feedforward_accumulated = 0.0f;
    g_gimbal.lock_count = 0;
    g_gimbal.target_lost_count = 0;
    g_gimbal.target_vx = 0.0f;
    g_gimbal.target_vy = 0.0f;
    g_outer_first_frame = true;

    GimbalCtrl_LaserOff();

    GimbalCtrl_SetRotationAngle(GIMBAL_ROTATION_NEUTRAL);
    GimbalCtrl_SetPitchAngle(GIMBAL_PITCH_NEUTRAL);

    g_gimbal.mode = GIMBAL_MODE_STOP;
}

/* ======================== 角度控制 ======================== */

void GimbalCtrl_SetRotationAngle(float angle)
{
    angle = LimitValue(angle, GIMBAL_ROTATION_MIN, GIMBAL_ROTATION_MAX);
    g_gimbal.current_rotation = angle;
    Motor_SetAngle(&g_rotation_motor, (int32_t)angle);
}

void GimbalCtrl_SetPitchAngle(float angle)
{
    angle = LimitValue(angle, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    g_gimbal.current_pitch = angle;
    GimbalCtrl_PitchNormal();  /* 自动唤醒 */
    Motor_SetAngle(&g_pitch_motor, (int32_t)angle);
}

/* ======================== 电机EMI控制 ======================== */

/* 俯仰电机低功耗：休眠 */
void GimbalCtrl_PitchLowPower(void)
{
    g_pitch_lowpower = true;
    g_pitch_sleep_tick = HAL_GetTick();

    /* 休眠释放扭矩（0x19 0x00），阻塞等待响应 */
    uint8_t ret = Motor_Sleep(&g_pitch_motor);
    if (ret != 1) {  /* 非0K */
        g_pitch_lowpower = false;
    }
}

/* 恢复俯仰电机正常工作 */
void GimbalCtrl_PitchNormal(void)
{
    if (!g_pitch_lowpower) return;
    g_pitch_lowpower = false;

    /* 唤醒（0x19 0x01），阻塞等待响应 */
    Motor_Wake(&g_pitch_motor);
}

/* ======================== 电机应用（内环输出） ======================== */

static void GimbalCtrl_ApplyMotors(float rot, float pitch)
{
    rot = LimitValue(rot, GIMBAL_ROTATION_MIN, GIMBAL_ROTATION_MAX);
    pitch = LimitValue(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

    g_gimbal.current_rotation = rot;
    g_gimbal.current_pitch = pitch;

    Motor_SetAngle(&g_rotation_motor, (int32_t)rot);

    /* 俯仰电机：手动休眠模式下不发命令，避免覆盖TX/RX缓冲 */
    if (g_manual_sleep) {
        return;
    }

    GimbalCtrl_PitchNormal();
    Motor_SetAngle(&g_pitch_motor, (int32_t)pitch);
}

/* 查询俯仰电机是否处于低功耗休眠中 */
bool GimbalCtrl_IsPitchSleeping(void)
{
    return g_pitch_lowpower;
}

/* 设置手动休眠标志 */
void GimbalCtrl_SetManualSleep(bool enabled)
{
    g_manual_sleep = enabled;
}

/* 查询手动休眠标志 */
bool GimbalCtrl_IsManualSleep(void)
{
    return g_manual_sleep;
}

/* 获取俯仰电机最近一次TX帧 */
uint8_t* GimbalCtrl_GetPitchTx(uint8_t *len)
{
    *len = 7;  /* 休眠/唤醒帧固定7字节 */
    return g_pitch_motor.tx_buf;
}

/* 获取俯仰电机最近一次RX响应 */
uint8_t* GimbalCtrl_GetPitchRx(uint8_t *len)
{
    *len = g_pitch_motor.rx_len;
    return g_pitch_motor.rx_buf;
}

/* ======================== 激光控制 ======================== */

void GimbalCtrl_LaserOn(void)
{
    LaserCtrl_On();
    g_gimbal.laser_on = true;
}

void GimbalCtrl_LaserOff(void)
{
    LaserCtrl_Off();
    g_gimbal.laser_on = false;
}

/* ======================== 获取器 ======================== */

float GimbalCtrl_GetRotationAngle(void)
{
    return g_gimbal.current_rotation;
}

float GimbalCtrl_GetPitchAngle(void)
{
    return g_gimbal.current_pitch;
}

GimbalCtrl_HandleTypeDef* GimbalCtrl_GetHandle(void)
{
    return &g_gimbal;
}


