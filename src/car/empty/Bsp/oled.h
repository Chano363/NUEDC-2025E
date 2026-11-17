#ifndef _OLED_H
#define _OLED_H

#include <stdint.h>

/**
 * @brief OLED 7-bit I2C address.
 * @note Most SSD1306/SSD1315 modules use 0x3C; change to 0x3D if SA0 is high.
 */
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR 0x3C
#endif

/**
 * @brief Initialize the SSD1306/SSD1315 128x64 OLED.
 * @note Software I2C: PA0=SCL and PA1=SDA (former KEY1/KEY2 pins).
 */
void OLED_Init(void);

/** @brief Clear the RAM buffer. Call OLED_Refresh() to update the panel. */
void OLED_Clear(void);

/**
 * @brief Draw a 6x8 ASCII string into the RAM buffer.
 * @param row Page number, 0..7.
 * @param col Character column, 0..20.
 * @param str Null-terminated ASCII string.
 */
void OLED_ShowString(uint8_t row, uint8_t col, const char *str);

/** @brief Draw a signed decimal integer into the RAM buffer. */
void OLED_ShowInt(uint8_t row, uint8_t col, int32_t num);

/** @brief Transfer the complete RAM buffer to the OLED. */
void OLED_Refresh(void);

#endif /* _OLED_H */
