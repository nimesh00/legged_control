#ifndef _XTERRA_LEGGED_SDK_
#define _XTERRA_LEGGED_SDK_

#include <stdbool.h>
#include <stdint.h>

namespace XTERRA_LEGGED_SDK {

// Different from the desired order: FL, FR, RL, RR
enum LEG_INDEX { FR_, FL_, RR_, RL_ };

#define XBOX_AXIS_COUNT 6     // LX, LY, RX, RY, LT, RT
#define XBOX_BUTTON_COUNT 14  // Total digital buttons

// Button bitmasks (aligned to standard XInput for compatibility)
enum XboxButton : uint16_t {
    XBOX_BTN_DPAD_UP = 1 << 0,
    XBOX_BTN_DPAD_DOWN = 1 << 1,
    XBOX_BTN_DPAD_LEFT = 1 << 2,
    XBOX_BTN_DPAD_RIGHT = 1 << 3,
    XBOX_BTN_START = 1 << 4,
    XBOX_BTN_BACK = 1 << 5,
    XBOX_BTN_LS = 1 << 6,  // Left stick press
    XBOX_BTN_RS = 1 << 7,  // Right stick press
    XBOX_BTN_LB = 1 << 8,  // Left bumper
    XBOX_BTN_RB = 1 << 9,  // Right bumper
    XBOX_BTN_A = 1 << 12,
    XBOX_BTN_B = 1 << 13,
    XBOX_BTN_X = 1 << 14,
    XBOX_BTN_Y = 1 << 15
};

typedef union {
    struct __attribute__((packed)) {  // Packed for consistent binary layout
        uint8_t priority;
        uint16_t buttons;  // Digital buttons (bitmask using enum above)
        uint8_t lt;        // Left trigger (0-255)
        uint8_t rt;        // Right trigger (0-255)
        int16_t lx;        // Left stick X (-32768 to 32767)
        int16_t ly;        // Left stick Y (-32768 to 32767)
        int16_t rx;        // Right stick X (-32768 to 32767)
        int16_t ry;        // Right stick Y (-32768 to 32767)
    } fields;
    uint8_t buffer[sizeof(uint16_t) * 2 + sizeof(uint8_t) * 2 +
                   sizeof(int16_t) * 4];  // Raw buffer (12 bytes)
} XboxJoystickState;

// Macro to check if a button is currently pressed (held down)
#define IS_BUTTON_PRESSED(state, button) \
    (((state).fields.buttons & (button)) != 0)

#define PRESS_BUTTON(state, button) ((state).fields.buttons |= (button))
#define RELEASE_BUTTON(state, button) ((state).fields.buttons &= ~(button))

}  // namespace XTERRA_LEGGED_SDK

#endif