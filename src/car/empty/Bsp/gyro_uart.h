#ifndef _GYRO_UART_H
#define _GYRO_UART_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float angle_deg;
    float dps;
    uint32_t frame_count;
    bool valid;
} GyroData_t;

typedef struct {
    bool initialized;
    uint8_t address;
    uint8_t who_am_i;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    float bias_dps;
} GyroStatus_t;

void Gyro_Init(void);
bool Gyro_GetData(GyroData_t *data);
void Gyro_CalibrateBias(void);
GyroStatus_t Gyro_GetStatus(void);
void Gyro_ProcessByte(uint8_t byte);

#endif
