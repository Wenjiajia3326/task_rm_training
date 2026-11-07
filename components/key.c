#include "key.h"
#include "bsp_board.h"

/// @brief 按下和松开的确认时间 单位ms
#define KEY_DEBOUNCE_MS 20

static KeyState key_state = KEY_RELEASED;
static uint32_t key_tick = 0;

uint8_t Key_Update(uint32_t now)
{
    GPIO_PinState level = BSP_Key_Read();

    switch (key_state)
    {
        case KEY_RELEASED:
            if (level == GPIO_PIN_RESET)
            {
                key_tick = now;
                key_state = KEY_PRESS_CHECK;
            }
            break;

        case KEY_PRESS_CHECK:
            if (now - key_tick >= KEY_DEBOUNCE_MS)
            {
                if (level == GPIO_PIN_RESET)
                {
                    key_state = KEY_PRESSED;
                    return 1;
                }

                key_state = KEY_RELEASED;
            }
            break;

        case KEY_PRESSED:
            if (level == GPIO_PIN_SET)
            {
                key_tick = now;
                key_state = KEY_RELEASE_CHECK;
            }
            break;

        case KEY_RELEASE_CHECK:
            if (now - key_tick >= KEY_DEBOUNCE_MS)
            {
                if (level == GPIO_PIN_SET)
                {
                    key_state = KEY_RELEASED;
                }
                else
                {
                    key_state = KEY_PRESSED;
                }
            }
            break;

        default:
            key_state = KEY_RELEASED;
            break;
    }

    return 0;
}
