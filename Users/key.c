#include "key.h"

/* ---- timing constants (ms) ---- */
#define KEY_DEBOUNCE_MS     25
#define KEY_LONG_PRESS_MS   2000
#define KEY_DOUBLE_CLICK_MS 250

/* ---- state machine ---- */
typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_DEBOUNCE,
    KEY_STATE_PRESSED,
    KEY_STATE_LONG_PRESS,
    KEY_STATE_RELEASE_WAIT,
    KEY_STATE_DEBOUNCE_2,
    KEY_STATE_PRESSED_2,
} key_state_t;

static key_state_t g_state = KEY_STATE_IDLE;
static uint32_t    g_entry_ms = 0;   /* 进入当前状态时的 tick */

/* ---- helper: read PA0 ---- */
static int key_is_pressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;
}

/* ---- public API ---- */

void KEY_Init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_initstruct.Pin   = GPIO_PIN_0;
    gpio_initstruct.Mode  = GPIO_MODE_INPUT;
    gpio_initstruct.Pull  = GPIO_NOPULL;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_initstruct);
}

int KEY_Scan(void)
{
    uint32_t now = HAL_GetTick();
    int pressed = key_is_pressed();
    uint32_t elapsed = now - g_entry_ms;

    switch (g_state) {

    /* ----- IDLE: 等待按下 ----- */
    case KEY_STATE_IDLE:
        if (pressed) {
            g_state    = KEY_STATE_DEBOUNCE;
            g_entry_ms = now;
        }
        break;

    /* ----- 初次消抖 (25ms) ----- */
    case KEY_STATE_DEBOUNCE:
        if (!pressed) {
            /* 噪声，退回 IDLE */
            g_state = KEY_STATE_IDLE;
        } else if (elapsed >= KEY_DEBOUNCE_MS) {
            /* 确认按下 */
            g_state    = KEY_STATE_PRESSED;
            g_entry_ms = now;
        }
        break;

    /* ----- 按下中，计时 ----- */
    case KEY_STATE_PRESSED:
        if (!pressed) {
            /* 松开 → 进入双击等待窗口 */
            g_state    = KEY_STATE_RELEASE_WAIT;
            g_entry_ms = now;
        } else if (elapsed >= KEY_LONG_PRESS_MS) {
            /* 长按 → 等释放 */
            g_state = KEY_STATE_LONG_PRESS;
            /* 立即返回长按事件 */
            return KEY_EVENT_LONG_PRESS;
        }
        break;

    /* ----- 长按后等释放 ----- */
    case KEY_STATE_LONG_PRESS:
        if (!pressed) {
            g_state = KEY_STATE_IDLE;
        }
        break;

    /* ----- 松开后 250ms 窗口 ----- */
    case KEY_STATE_RELEASE_WAIT:
        if (pressed) {
            /* 250ms 内有第二次按下 → 消抖 */
            g_state    = KEY_STATE_DEBOUNCE_2;
            g_entry_ms = now;
        } else if (elapsed >= KEY_DOUBLE_CLICK_MS) {
            /* 超时 → 确认为单击 */
            g_state = KEY_STATE_IDLE;
            return KEY_EVENT_CLICK;
        }
        break;

    /* ----- 第二次消抖 (25ms) ----- */
    case KEY_STATE_DEBOUNCE_2:
        if (!pressed) {
            /* 噪声 → 退回等待窗口 (剩余时间继续) */
            g_state = KEY_STATE_RELEASE_WAIT;
        } else if (elapsed >= KEY_DEBOUNCE_MS) {
            /* 确认第二次按下 */
            g_state    = KEY_STATE_PRESSED_2;
            g_entry_ms = now;
        }
        break;

    /* ----- 第二次按下，等释放 → 双击 ----- */
    case KEY_STATE_PRESSED_2:
        if (!pressed) {
            g_state = KEY_STATE_IDLE;
            return KEY_EVENT_DOUBLE_CLICK;
        }
        break;

    default:
        g_state = KEY_STATE_IDLE;
        break;
    }

    return KEY_EVENT_NONE;
}
