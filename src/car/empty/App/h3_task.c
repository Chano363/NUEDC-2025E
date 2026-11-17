#include "h3_task.h"
#include "h3_config.h"
#include "heading_ctrl.h"
#include "gyro_fusion.h"
#include "tracking.h"
#include "Bsp/grayscale.h"
#include "Bsp/key.h"
#include "Bsp/motor_drv.h"
#include "Lib/math_utils.h"
#include "ti_msp_dl_config.h"

#define H3_STRAIGHT_SPEED          H3_CFG_STRAIGHT_SPEED
#define H3_TURN_SPEED              H3_CFG_TURN_SPEED
#define H3_MIN_FORWARD_SPEED       H3_CFG_MIN_FORWARD_SPEED
#define H3_STRAIGHT_MAX_CORRECTION H3_CFG_STRAIGHT_MAX_CORRECTION
#define H3_TURN_MAX_CORRECTION     H3_CFG_TURN_MAX_CORRECTION
#define H3_DIAGONAL_ANGLE_DEG      H3_CFG_DIAGONAL_ANGLE_DEG
#define H3_MODE_C_INITIAL_HEADING_OFFSET_DEG \
    H3_CFG_MODE_C_INITIAL_HEADING_OFFSET_DEG
#define H3_MODE_C_CB_EXIT_HEADING_DEG \
    H3_CFG_MODE_C_CB_EXIT_HEADING_DEG
#define H3_MODE_C_DA_EXIT_HEADING_DEG \
    H3_CFG_MODE_C_DA_EXIT_HEADING_DEG
#define H3_LINE_CLEAR_TICKS        H3_CFG_LINE_CLEAR_TICKS
#define H3_LINE_HIT_TICKS          H3_CFG_LINE_HIT_TICKS
#define H3_ARC_LOST_TICKS          H3_CFG_ARC_LOST_TICKS
#define H3_MIN_STRAIGHT_TIME_MS    H3_CFG_MIN_STRAIGHT_TIME_MS
#define H3_MIN_ARC_TIME_MS         H3_CFG_MIN_ARC_TIME_MS
#define H3_MIN_ARC_HEADING_DEG     H3_CFG_MIN_ARC_HEADING_DEG
#define H3_MODE_B_ARC_HEADING_DEG  H3_CFG_MODE_B_ARC_HEADING_DEG
#define H3_ARC_CENTER_STABLE_TICKS H3_CFG_ARC_CENTER_STABLE_TICKS
#define H3_MODE_B_ARC_LOST_EXIT_TICKS H3_CFG_MODE_B_ARC_LOST_EXIT_TICKS
#define H3_ALIGN_SETTLE_MS         H3_CFG_ALIGN_SETTLE_MS
#define H3_ARC_EXIT_STRAIGHT_MS    H3_CFG_ARC_EXIT_STRAIGHT_MS
#define H3_MODE_C_LAPS             H3_CFG_MODE_C_LAPS
#define H3_TIMEOUT_MS              H3_CFG_TIMEOUT_MS
#define H3_SIGNAL_MS               H3_CFG_SIGNAL_MS
#define H3_RELEASE_DELAY_MS        H3_CFG_RELEASE_DELAY_MS

/* -------------------------------------------------------------------------
 * H3 state machine data.  Control algorithms live in tracking/heading_ctrl;
 * this file only decides which controller owns the motors at each phase.
 * ------------------------------------------------------------------------- */
static H3_OutputTypeDef g_output;
static uint32_t g_task_start;
static uint32_t g_state_start;
static uint32_t g_signal_off;
static uint32_t g_release_start;
static float g_target_heading;
static float g_arc_start_heading;
static float g_exit_straight_heading;
static float g_align_target_heading;
static bool g_align_to_complete;
static bool g_align_to_diagonal;
static bool g_align_to_next_lap;
static uint8_t g_clear_count;
static uint8_t g_hit_count;
static uint8_t g_lost_count;
static uint8_t g_arc_center_count;
static uint8_t g_lap_count;
static bool g_origin_clear;
static H3_ModeTypeDef g_mode = H3_MODE_A;

/* User feedback ---------------------------------------------------------- */
static void signal_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(USER_IO_PORT, USER_IO_STATUS_LED_PIN);
        DL_GPIO_setPins(USER_IO_PORT, USER_IO_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(USER_IO_PORT, USER_IO_STATUS_LED_PIN);
        DL_GPIO_clearPins(USER_IO_PORT, USER_IO_BUZZER_PIN);
    }
}

static void signal_point(uint32_t now)
{
    signal_set(true);
    g_signal_off = now + H3_SIGNAL_MS;
}

static void set_state(H3_StateTypeDef state, uint32_t now)
{
    g_output.state = state;
    g_state_start = now;
    g_clear_count = 0U;
    g_hit_count = 0U;
    g_lost_count = 0U;
    g_arc_center_count = 0U;
}

static void stop_state(H3_StateTypeDef state, uint32_t now)
{
    Tracking_Stop();
    Tracking_SetArcMode(false);
    g_output.target_left = 0;
    g_output.target_right = 0;
    g_output.running = false;
    g_output.finished = (state == H3_STATE_COMPLETE);
    set_state(state, now);
    signal_point(now);
}

/* Straight endpoint detection ------------------------------------------- */
static void read_line(void)
{
    Grayscale_DataDef line;
    Grayscale_Update(&line);
    g_output.digital_status = line.digital_status;
    g_output.deviation = line.deviation;
}

static bool target_line_reached(uint32_t now)
{
    if (g_output.digital_status == 0U) {
        if (g_clear_count < UINT8_MAX) g_clear_count++;
        g_hit_count = 0U;
        if (g_clear_count >= H3_LINE_CLEAR_TICKS) g_origin_clear = true;
    } else if (g_origin_clear
               && now - g_state_start >= H3_MIN_STRAIGHT_TIME_MS) {
        g_clear_count = 0U;
        if (g_hit_count < UINT8_MAX) g_hit_count++;
    }
    return g_hit_count >= H3_LINE_HIT_TICKS;
}

static void run_heading(float target, int16_t base, int16_t limit)
{
    HeadingCtrl_Hold(target, base, limit, &g_output.target_left,
                       &g_output.target_right, &g_output.heading_error_deg);
}

static void run_fixed_turn(float target)
{
    HeadingCtrl_AlignInPlace(target, H3_TURN_MAX_CORRECTION,
                              H3_CFG_ALIGN_MIN_PWM,
                              &g_output.target_left, &g_output.target_right,
                         &g_output.heading_error_deg);
}

static void run_forward_turn(float target)
{
    HeadingCtrl_TurnTo(target, H3_TURN_SPEED, H3_TURN_MAX_CORRECTION,
                           &g_output.target_left, &g_output.target_right,
                           &g_output.heading_error_deg);
}

static void begin_arc_alignment(uint32_t now, bool finish_after_align,
                                bool turn_to_diagonal)
{
    /* 丢线瞬间先记住车辆当前航向。出弯缓冲段保持这个航向，避免在
     * 车身尚未完全离开弯道时提前强行转到 0°/180°。 */
    g_exit_straight_heading = GyroFusion_GetHeading();
    float direction = (GyroFusion_GetHeading() >= g_arc_start_heading)
        ? 1.0f : -1.0f;
    Tracking_Stop();
    Tracking_SetArcMode(false);
    if (g_mode == H3_MODE_C && finish_after_align) {
        g_align_target_heading = H3_MODE_C_DA_EXIT_HEADING_DEG;
    } else if (g_mode == H3_MODE_C && turn_to_diagonal && !finish_after_align) {
        /* C→B must finish on the fixed horizontal -180-degree heading. */
        g_align_target_heading = H3_MODE_C_CB_EXIT_HEADING_DEG;
    } else {
        g_align_target_heading = g_arc_start_heading + direction * 180.0f;
    }
    g_align_to_complete = finish_after_align;
    g_align_to_diagonal = turn_to_diagonal;
    /* 先用出弯目标航向直行一小段，让车身真正离开半圆；之后再做
     * 原地姿态校正或固定角度转向。 */
    if (H3_ARC_EXIT_STRAIGHT_MS > 0U) {
        set_state(H3_STATE_ARC_EXIT_STRAIGHT, now);
    } else {
        set_state(H3_STATE_ARC_ALIGN, now);
    }
}

static void apply_tracking_output(const Tracking_OutputDef *tracking)
{
    g_output.target_left = (int16_t)LimitIntValue(tracking->target_left,
        H3_MIN_FORWARD_SPEED, MOTOR_PWM_MAX);
    g_output.target_right = (int16_t)LimitIntValue(tracking->target_right,
        H3_MIN_FORWARD_SPEED, MOTOR_PWM_MAX);
    g_output.digital_status = Tracking_GetDigitalStatus();
    g_output.deviation = Tracking_GetDeviation();
}

/* Semicircle endpoint detection ----------------------------------------- */
static bool arc_reached(uint32_t now)
{
    /* 航向角是累计值，不能用绝对航向判断半圆进度；必须比较本次
     * 半圆相对入弯时的航向变化，否则第二个半圆开始时可能已经超过
     * 120°，刚进入循迹就被误判为出弯。 */
    float arc_heading_delta = AbsFloat(
        GyroFusion_GetHeading() - g_arc_start_heading);

    if (g_mode == H3_MODE_B) {
        /* A BC/DA semicircle is complete only after the car has rotated
         * close to 180 degrees and the center sensors have reacquired the
         * straight black line. A short line dropout during the bend must
         * not switch to the next straight segment prematurely. */
        if (now - g_state_start < H3_MIN_ARC_TIME_MS
            || arc_heading_delta < H3_MODE_B_ARC_HEADING_DEG) {
            g_arc_center_count = 0U;
            g_lost_count = 0U;
            return false;
        }
        if ((g_output.digital_status & 0x18U) != 0U
            && AbsFloat((float)g_output.deviation) <= 10.0f) {
            if (g_arc_center_count < UINT8_MAX) g_arc_center_count++;
            g_lost_count = 0U;
        } else {
            g_arc_center_count = 0U;
            if (g_output.digital_status == 0U) {
                if (g_lost_count < UINT8_MAX) g_lost_count++;
            } else {
                g_lost_count = 0U;
            }
        }
        /* Once the car has already rotated to the exit heading, a real
         * straight-line transition can present as a short line dropout.
         * Accept that dropout after a few ticks; otherwise arc-mode's
         * keep-turning lost-line rule would make the turn progressively
         * sharper instead of handing control to the heading loop. */
        return g_arc_center_count >= H3_ARC_CENTER_STABLE_TICKS
            || g_lost_count >= H3_MODE_B_ARC_LOST_EXIT_TICKS;
    }

    if (g_output.digital_status == 0U) {
        if (g_lost_count < UINT8_MAX) g_lost_count++;
    } else {
        g_lost_count = 0U;
    }
    if (now - g_state_start < H3_MIN_ARC_TIME_MS
        || g_lost_count < H3_ARC_LOST_TICKS) return false;
    return !GyroFusion_IsValid()
        || arc_heading_delta >= H3_MIN_ARC_HEADING_DEG;
}

/* Public mode/state API -------------------------------------------------- */
void H3_Init(void)
{
    g_output = (H3_OutputTypeDef){0};
    g_output.state = H3_STATE_IDLE;
    g_signal_off = 0U;
    g_mode = H3_MODE_A;
    signal_set(false);
}

void H3_SetMode(H3_ModeTypeDef mode)
{
    if (g_output.running) return;
    if (mode > H3_MODE_C) mode = H3_MODE_A;
    g_mode = mode;
    g_output.finished = false;
    g_output.state = H3_STATE_IDLE;
}

H3_ModeTypeDef H3_GetMode(void)
{
    return g_mode;
}

void H3_CycleMode(void)
{
    if (g_output.running) return;
    g_mode = (H3_ModeTypeDef)((g_mode + 1U) % 3U);
    g_output.finished = false;
    g_output.state = H3_STATE_IDLE;
}

void H3_Start(uint32_t now)
{
    Tracking_Stop();
    Tracking_SetArcMode(false);
    g_task_start = now;
    g_target_heading = 0.0f;
    g_origin_clear = false;
    g_release_start = 0U;
    g_lap_count = 0U;
    g_align_to_next_lap = false;
    g_output.running = true;
    g_output.finished = false;
    set_state(H3_STATE_ARMED, now);
    signal_point(now);
}

/* Main 20 ms state-machine step.  Each state has one motor owner:
 * STRAIGHT_* -> heading_ctrl, ARC_* -> tracking, ARC_ALIGN/TURN ->
 * heading_ctrl angle operation. */
void H3_Update(uint32_t now)
{
    Tracking_OutputDef tracking;
    if (g_signal_off != 0U && (int32_t)(now - g_signal_off) >= 0) {
        signal_set(false);
        g_signal_off = 0U;
    }
    if (!g_output.running) {
        read_line();
        return;
    }
    if (now - g_task_start > H3_TIMEOUT_MS) {
        stop_state(H3_STATE_ERROR_TIMEOUT, now);
        return;
    }

    switch (g_output.state) {
    case H3_STATE_ARMED:
        g_output.target_left = 0;
        g_output.target_right = 0;
        read_line();
        if (Key_IsPressed(KEY_ID_START)) {
            g_release_start = 0U;
        } else if (g_release_start == 0U) {
            g_release_start = now;
        } else if (now - g_release_start >= H3_RELEASE_DELAY_MS) {
            GyroFusion_Zero();
            if (!GyroFusion_IsValid()) {
                /* Keep the car stopped and wait for the next valid frame. */
                g_release_start = now;
                break;
            }
            g_origin_clear = false;
            g_target_heading = (g_mode == H3_MODE_C)
                ? H3_MODE_C_INITIAL_HEADING_OFFSET_DEG : 0.0f;
            if (g_mode == H3_MODE_C) {
                /* The gyro was zeroed along AB. First turn in place to the
                 * configured AC diagonal, then start the A->C straight. */
                g_align_target_heading = g_target_heading;
                set_state(H3_STATE_START_ALIGN, now);
            } else {
                set_state(H3_STATE_STRAIGHT_AC, now);
            }
        }
        break;
    case H3_STATE_START_ALIGN:
        run_fixed_turn(g_align_target_heading);
        if (HeadingCtrl_IsSettled(g_align_target_heading, 3.0f, 15.0f)
            && now - g_state_start >= H3_ALIGN_SETTLE_MS) {
            g_target_heading = g_align_target_heading;
            g_origin_clear = false;
            set_state(H3_STATE_STRAIGHT_AC, now);
        }
        break;
    case H3_STATE_STRAIGHT_AC:
        read_line();
        run_heading(g_target_heading, H3_STRAIGHT_SPEED, H3_STRAIGHT_MAX_CORRECTION);
        if (target_line_reached(now)) {
            if (g_mode == H3_MODE_A) {
                stop_state(H3_STATE_COMPLETE, now);
                break;
            }
            signal_point(now);
            Tracking_SetArcMode(true);
            Tracking_Start(1U);
            g_arc_start_heading = GyroFusion_GetHeading();
            /* Take the first arc-tracking sample immediately.  Do not leave
             * the straight heading PWM active for the transition tick. */
            Tracking_Update();
            tracking = Tracking_GetOutput();
            apply_tracking_output(&tracking);
            set_state(H3_STATE_ARC_CB, now);
        }
        break;
    case H3_STATE_ARC_CB:
        Tracking_Update();
        tracking = Tracking_GetOutput();
        apply_tracking_output(&tracking);
        if (arc_reached(now)) {
            signal_point(now);
            if (g_mode == H3_MODE_B) {
                /* First square the chassis to the geometric 180-degree
                 * exit heading; only then enable straight heading hold. */
                begin_arc_alignment(now, false, false);
            } else {
                /* Mode C uses the same 180-degree squaring first; the 53
                 * degree diagonal turn is started only after ARC_ALIGN. */
                begin_arc_alignment(now, false, true);
            }
        }
        break;
    case H3_STATE_TURN_BD:
        run_forward_turn(g_target_heading);
        if (HeadingCtrl_IsSettled(g_target_heading, 3.0f, 15.0f)) {
            if (now - g_state_start >= 100U) {
                g_origin_clear = false;
                set_state(H3_STATE_STRAIGHT_BD, now);
            }
        }
        break;
    case H3_STATE_STRAIGHT_BD:
        read_line();
        run_heading(g_target_heading, H3_STRAIGHT_SPEED, H3_STRAIGHT_MAX_CORRECTION);
        if (target_line_reached(now)) {
            signal_point(now);
            Tracking_SetArcMode(true);
            Tracking_Start(1U);
            g_arc_start_heading = GyroFusion_GetHeading();
            Tracking_Update();
            tracking = Tracking_GetOutput();
            apply_tracking_output(&tracking);
            set_state(H3_STATE_ARC_DA, now);
        }
        break;
    case H3_STATE_ARC_DA:
        Tracking_Update();
        tracking = Tracking_GetOutput();
        apply_tracking_output(&tracking);
        if (arc_reached(now)) {
            begin_arc_alignment(now, true, false);
        }
        break;
    case H3_STATE_ARC_EXIT_STRAIGHT:
        /* 出弯缓冲段保持丢线瞬间的原航向直行；缓冲结束后才进入
         * ARC_ALIGN，将姿态校正到对应点位的整数航向。 */
        run_heading(g_exit_straight_heading, H3_STRAIGHT_SPEED,
                    H3_STRAIGHT_MAX_CORRECTION);
        if (now - g_state_start >= H3_ARC_EXIT_STRAIGHT_MS) {
            set_state(H3_STATE_ARC_ALIGN, now);
        }
        break;
    case H3_STATE_ARC_ALIGN:
        run_fixed_turn(g_align_target_heading);
        if (HeadingCtrl_IsSettled(g_align_target_heading, 3.0f, 15.0f)) {
            if (now - g_state_start >= H3_ALIGN_SETTLE_MS) {
                if (g_align_to_next_lap) {
                    /* DA 出弯校正完成后，下一圈必须明确回到 AC 航向；
                     * 不能直接沿 DA 的对向航向进入下一段直线。 */
                    g_align_to_next_lap = false;
                    g_align_to_complete = false;
                    g_target_heading = H3_MODE_C_INITIAL_HEADING_OFFSET_DEG;
                    g_align_target_heading = g_target_heading;
                    g_origin_clear = false;
                    set_state(H3_STATE_STRAIGHT_AC, now);
                } else if (g_align_to_complete) {
                    if (g_mode == H3_MODE_C) {
                        /* DA 到 A 算完成一圈；模式 C 继续下一圈，满四圈停车。 */
                        if (g_lap_count < UINT8_MAX) g_lap_count++;
                        if (g_lap_count < H3_MODE_C_LAPS) {
                            /* 下一圈先切换到 AC 航向，再开始直线。 */
                            g_align_to_next_lap = false;
                            g_align_to_complete = false;
                            g_align_target_heading =
                                H3_MODE_C_INITIAL_HEADING_OFFSET_DEG;
                            set_state(H3_STATE_START_ALIGN, now);
                        } else {
                            stop_state(H3_STATE_COMPLETE, now);
                        }
                    } else {
                        stop_state(H3_STATE_COMPLETE, now);
                    }
                } else if (g_align_to_diagonal) {
                    /* The chassis is now exactly opposite the arc entry
                     * heading. Only after this correction do we make the
                     * fixed diagonal turn toward the destination. */
                    /* From -180 degrees, B→D uses the positive 53-degree
                     * turn in this coordinate convention. */
                    g_target_heading = g_align_target_heading
                        - H3_DIAGONAL_ANGLE_DEG;
                    set_state(H3_STATE_TURN_BD, now);
                } else {
                    g_target_heading = g_align_target_heading;
                    g_origin_clear = false;
                    set_state(H3_STATE_STRAIGHT_BD, now);
                }
            }
        }
        break;
    default:
        stop_state(H3_STATE_ERROR_TIMEOUT, now);
        break;
    }
}

H3_OutputTypeDef H3_GetOutput(void)
{
    H3_OutputTypeDef output = g_output;
    output.lap_count = g_lap_count;
    output.target_heading_deg =
        (g_output.state == H3_STATE_ARC_EXIT_STRAIGHT)
            ? g_exit_straight_heading
            : ((g_output.state == H3_STATE_ARC_ALIGN)
                ? g_align_target_heading : g_target_heading);
    return output;
}

char H3_GetStateCode(void)
{
    static const char code[] = {'I','K','S','C','B','T','D','A','P','R','E','G','X'};
    return ((uint8_t)g_output.state < sizeof(code)) ? code[g_output.state] : '?';
}
