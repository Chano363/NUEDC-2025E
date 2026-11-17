#include "k230_uart.h"
#include "main.h"
#include "usart.h"
#include "../Drv/drv_uart.h"
#include <string.h>

/* K230协议帧格式 (接收): [0xAA] [0xFF] [func_code] [length] [data...] [sc] [ac] */
#define K230_RX_HEADER1     0xAA
#define K230_RX_HEADER2     0xFF

/* K230协议帧格式 (发送): [0xAA] [0xFE] [cmd] [length] [data...] [sc] [ac] */
#define K230_TX_HEADER1     0xAA
#define K230_TX_HEADER2     0xFE

#define K230_MAX_PAYLOAD    128
#define K230_TARGET_TIMEOUT_MS  200U

/* 解析状态机 */
typedef enum {
    K230_STATE_IDLE,
    K230_STATE_HEADER2,    /* 已收到0xAA，等待0xFF */
    K230_STATE_FUNC,
    K230_STATE_LENGTH,
    K230_STATE_PAYLOAD,
    K230_STATE_SC,
    K230_STATE_AC
} K230_ParseState;

static K230_ParseState g_parse_state = K230_STATE_IDLE;
static uint8_t g_func_code = 0;
static uint8_t g_frame_length = 0;
static uint8_t g_payload[K230_MAX_PAYLOAD];
static uint8_t g_payload_idx = 0;
static uint8_t g_sc = 0;

/* 解析后的数据 */
static K230_TargetData    g_target_data = {0};
static K230_LaserData     g_laser_data = {0};
static K230_DeviationData g_deviation_data = {0};
static K230_HitData       g_hit_data = {0};

/* 连接状态 */
static bool     g_connected = false;
static uint32_t g_last_receive_time = 0;
static uint32_t g_last_target_time = 0;
static uint32_t g_last_laser_time = 0;

/* 调试计数 */
volatile uint32_t g_k230_rx_bytes = 0;      /* 收到的原始字节数 */
volatile uint32_t g_k230_valid_frames = 0;  /* 校验通过的帧数 */

/* 小端16位读取 */
static int16_t read_int16_le(const uint8_t *buf)
{
    return (int16_t)(buf[0] | (buf[1] << 8));
}

static uint16_t read_uint16_le(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

/* 计算双校验和: sc = 逐字节累加和, ac = 逐字节累加sc */
static void calc_dual_checksum(const uint8_t *data, uint8_t len, uint8_t *out_sc, uint8_t *out_ac)
{
    uint8_t sc = 0, ac = 0;
    for (uint8_t i = 0; i < len; i++) {
        sc = (sc + data[i]) & 0xFF;
        ac = (ac + sc) & 0xFF;
    }
    *out_sc = sc;
    *out_ac = ac;
}

void K230_UART_Init(void)
{
    g_parse_state = K230_STATE_IDLE;
    g_func_code = 0;
    g_frame_length = 0;
    g_payload_idx = 0;
    g_sc = 0;

    memset(&g_target_data, 0, sizeof(g_target_data));
    memset(&g_laser_data, 0, sizeof(g_laser_data));
    memset(&g_deviation_data, 0, sizeof(g_deviation_data));
    memset(&g_hit_data, 0, sizeof(g_hit_data));

    g_target_data.valid = false;
    g_laser_data.valid = false;
    g_deviation_data.valid = false;
    g_hit_data.valid = false;
    g_connected = false;
    g_last_target_time = 0;
    g_last_laser_time = 0;
}

void K230_UART_Process(void)
{
    uint8_t byte;

    while (DrvUART_Available(&huart1)) {
        if (!DrvUART_Read(&huart1, &byte)) break;

        g_k230_rx_bytes++;
        g_connected = true;
        g_last_receive_time = HAL_GetTick();

        switch (g_parse_state) {
            case K230_STATE_IDLE:
                if (byte == K230_RX_HEADER1) {
                    g_parse_state = K230_STATE_HEADER2;
                }
                break;

            case K230_STATE_HEADER2:
                if (byte == K230_RX_HEADER2) {
                    g_parse_state = K230_STATE_FUNC;
                } else if (byte == K230_RX_HEADER1) {
                    /* 连续0xAA，继续等待0xFF */
                } else {
                    g_parse_state = K230_STATE_IDLE;
                }
                break;

            case K230_STATE_FUNC:
                g_func_code = byte;
                g_parse_state = K230_STATE_LENGTH;
                break;

            case K230_STATE_LENGTH:
                g_frame_length = byte;
                g_payload_idx = 0;
                if (g_frame_length > K230_MAX_PAYLOAD) {
                    g_parse_state = K230_STATE_IDLE;
                    break;
                }
                if (g_frame_length == 0) {
                    g_parse_state = K230_STATE_SC;
                } else {
                    g_parse_state = K230_STATE_PAYLOAD;
                }
                break;

            case K230_STATE_PAYLOAD:
                if (g_payload_idx < K230_MAX_PAYLOAD) {
                    g_payload[g_payload_idx++] = byte;
                }
                if (g_payload_idx >= g_frame_length) {
                    g_parse_state = K230_STATE_SC;
                }
                break;

            case K230_STATE_SC:
                g_sc = byte;
                g_parse_state = K230_STATE_AC;
                break;

            case K230_STATE_AC: {
                /* 校验: 对 [func_code, data...] 计算双校验和（与K230侧一致） */
                uint8_t check_buf[K230_MAX_PAYLOAD + 4];
                check_buf[0] = g_func_code;
                memcpy(&check_buf[1], g_payload, g_payload_idx);

                uint8_t calc_sc, calc_ac;
                calc_dual_checksum(check_buf, 1 + g_payload_idx, &calc_sc, &calc_ac);

                if (calc_sc == g_sc && calc_ac == byte) {
                    g_k230_valid_frames++;
                    /* 校验通过，解析数据 */
                    switch (g_func_code) {
                        case K230_FUNC_PURPLE_BLOB:
                            /* 紫色色块: x_low, x_high, y_low, y_high */
                            if (g_payload_idx >= 4) {
                                g_laser_data.x = read_int16_le(&g_payload[0]);
                                g_laser_data.y = read_int16_le(&g_payload[2]);
                                g_laser_data.valid = true;
                                g_laser_data.frame_count++;
                                g_last_laser_time = HAL_GetTick();
                            }
                            break;

                        case K230_FUNC_TARGET_CENTER:
                            /* 目标圆心: x_low, x_high, y_low, y_high */
                            if (g_payload_idx >= 4) {
                                g_target_data.x = read_int16_le(&g_payload[0]);
                                g_target_data.y = read_int16_le(&g_payload[2]);
                                g_target_data.valid = true;
                                g_target_data.frame_count++;
                                g_last_target_time = HAL_GetTick();
                            }
                            break;

                        case K230_FUNC_CIRCLE_POINTS:
                            /* 圆形点集: 每4字节一个点(x_low, x_high, y_low, y_high) */
                            /* 暂存到target_data作为圆心近似，详细使用按需扩展 */
                            break;

                        case K230_FUNC_DEVIATION:
                            /* 偏差: dx_low, dx_high, dy_low, dy_high */
                            if (g_payload_idx >= 4) {
                                g_deviation_data.dx = read_int16_le(&g_payload[0]);
                                g_deviation_data.dy = read_int16_le(&g_payload[2]);
                                g_deviation_data.valid = true;
                                g_deviation_data.frame_count++;
                            }
                            break;

                        case K230_FUNC_HIT_CONFIRM:
                            /* 命中确认: distance_low, distance_high, confidence */
                            if (g_payload_idx >= 3) {
                                g_hit_data.distance = read_uint16_le(&g_payload[0]);
                                g_hit_data.confidence = g_payload[2];
                                g_hit_data.valid = true;
                                g_hit_data.frame_count++;
                            }
                            break;

                        default:
                            break;
                    }
                }
                g_parse_state = K230_STATE_IDLE;
                break;
            }

            default:
                g_parse_state = K230_STATE_IDLE;
                break;
        }
    }

    /* 超时检测 */
    if (HAL_GetTick() - g_last_receive_time > 2000) {
        g_connected = false;
        g_target_data.valid = false;
        g_laser_data.valid = false;
        g_deviation_data.valid = false;
        g_hit_data.valid = false;
    }
}

K230_TargetData K230_UART_GetTargetData(void)
{
    K230_TargetData data = g_target_data;
    if (data.valid && (HAL_GetTick() - g_last_target_time > K230_TARGET_TIMEOUT_MS)) {
        data.valid = false;
    }
    return data;
}

K230_LaserData K230_UART_GetLaserData(void)
{
    K230_LaserData data = g_laser_data;
    if (data.valid && (HAL_GetTick() - g_last_laser_time > K230_TARGET_TIMEOUT_MS)) {
        data.valid = false;
    }
    return data;
}

K230_DeviationData K230_UART_GetDeviationData(void)
{
    return g_deviation_data;
}

K230_HitData K230_UART_GetHitData(void)
{
    return g_hit_data;
}

bool K230_UART_IsConnected(void)
{
    return g_connected;
}

/* 发送命令到K230: [0xAA] [0xFE] [cmd] [length] [data...] [sc] [ac] */
void K230_UART_SendCommand(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (len > K230_MAX_PAYLOAD) return;

    uint8_t frame[K230_MAX_PAYLOAD + 7];
    uint8_t idx = 0;

    frame[idx++] = K230_TX_HEADER1;   /* 0xAA */
    frame[idx++] = K230_TX_HEADER2;   /* 0xFE */
    frame[idx++] = cmd;               /* 命令码 */
    frame[idx++] = len;               /* 数据长度 */

    if (len > 0 && data != NULL) {
        memcpy(&frame[idx], data, len);
        idx += len;
    }

    /* 计算双校验和: 仅对 [cmd, data...] 计算（与K230侧一致，不包含length） */
    uint8_t check_buf[K230_MAX_PAYLOAD + 1];
    check_buf[0] = cmd;
    if (len > 0 && data != NULL) {
        memcpy(&check_buf[1], data, len);
    }
    uint8_t calc_sc, calc_ac;
    calc_dual_checksum(check_buf, 1 + len, &calc_sc, &calc_ac);
    frame[idx++] = calc_sc;
    frame[idx++] = calc_ac;

    DrvUART_Send(&huart1, frame, idx);
}

void K230_UART_SendStartDetection(uint8_t mode)
{
    K230_UART_SendCommand(K230_CMD_START, &mode, 1);
}

void K230_UART_SendStopDetection(void)
{
    K230_UART_SendCommand(K230_CMD_STOP, NULL, 0);
}
