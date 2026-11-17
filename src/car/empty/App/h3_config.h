#ifndef H3_CONFIG_H
#define H3_CONFIG_H

/* H3 任务调参参数。
 * 单位说明：速度/限幅为 PWM，时间为 ms，角度为度。
 * 实车调试时优先修改本文件。 */

/* 直线段参数：直线航向保持由 heading_ctrl 完成。 */
#define H3_CFG_STRAIGHT_SPEED             100
#define H3_CFG_STRAIGHT_MAX_CORRECTION    55
#define H3_CFG_MIN_FORWARD_SPEED          10

/* 固定角度转向（模式 C）和出弯原地姿态校正（模式 B）。 */
#define H3_CFG_TURN_SPEED                 85
#define H3_CFG_TURN_MAX_CORRECTION        45
#define H3_CFG_DIAGONAL_ANGLE_DEG         52.73f
/* 模式 C 的 A→C 初始航向偏置：
 * 零点沿 AC 标定时设为 0；零点沿 AB 标定时按实车转向方向设为 ±53°。 */
#define H3_CFG_MODE_C_INITIAL_HEADING_OFFSET_DEG 52.73f
/* 模式 C 中 C→B 半圆出弯的水平对岸航向。 */
#define H3_CFG_MODE_C_CB_EXIT_HEADING_DEG       -180.0f
/* 模式 C 的 DA 出弯固定航向；当前坐标系下 A 点使用 0°。 */
#define H3_CFG_MODE_C_DA_EXIT_HEADING_DEG          0.0f
#define H3_CFG_ALIGN_MIN_PWM              65
#define H3_CFG_ALIGN_SETTLE_MS            100U

/* 半圆出弯后，灰度探头已经丢线但车身尚未完全离开弯道。
 * 这段时间保持出弯整数航向直行，再进入原地姿态校正/转向。
 * 单位 ms，按实车车速和车身长度调节；设为 0 可关闭。 */
#define H3_CFG_ARC_EXIT_STRAIGHT_MS       450U

/* 模式 C 的完整路线圈数：A→C→B→D→A。 */
#define H3_CFG_MODE_C_LAPS                4U

/* 直线终点检测：先确认离开原黑线，再确认重新压上线。 */
#define H3_CFG_LINE_CLEAR_TICKS           5U
#define H3_CFG_LINE_HIT_TICKS             2U
#define H3_CFG_MIN_STRAIGHT_TIME_MS       400U

/* 半圆出弯检测：模式 B 使用航向角和中间传感器共同判断；模式 C
 * 保留第三问流程中的丢线/航向角备用判定。 */
#define H3_CFG_MIN_ARC_TIME_MS            1200U
#define H3_CFG_MIN_ARC_HEADING_DEG        120.0f
#define H3_CFG_MODE_B_ARC_HEADING_DEG     160.0f
#define H3_CFG_ARC_CENTER_STABLE_TICKS    5U
#define H3_CFG_MODE_B_ARC_LOST_EXIT_TICKS 3U
#define H3_CFG_ARC_LOST_TICKS             4U

/* 通用安全保护和人机交互时序参数。 */
#define H3_CFG_TIMEOUT_MS                 45000U
#define H3_CFG_SIGNAL_MS                  300U
#define H3_CFG_RELEASE_DELAY_MS           300U

#endif
