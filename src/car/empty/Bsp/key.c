#include "key.h"

static Key_StateTypeDef g_key_state[KEY_ID_MAX];

void Key_Init(void)
{
    for (uint8_t i = 0; i < KEY_ID_MAX; i++) {
        g_key_state[i] = KEY_STATE_RELEASED;
    }
}

void Key_Update(void)
{
    /* Key GPIOs are intentionally disabled; PA0/PA1 are used by OLED I2C. */
}

bool Key_IsPressed(Key_IDTypeDef key_id)
{
    return (key_id < KEY_ID_MAX) && (g_key_state[key_id] == KEY_STATE_PRESSED);
}

Key_StateTypeDef Key_GetState(Key_IDTypeDef key_id)
{
    if (key_id >= KEY_ID_MAX) return KEY_STATE_RELEASED;
    return g_key_state[key_id];
}
