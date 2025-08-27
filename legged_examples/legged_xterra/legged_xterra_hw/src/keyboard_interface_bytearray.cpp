#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "ByteArray.hpp"  // Assuming the generated DDS type for ByteArray_
#include "dds/dds_publisher.hpp"
#include "xterra_legged_sdk.h"  // Assuming this includes XboxJoystickState, macros, enums

using namespace xterra::msg::dds_;
using namespace XTERRA_LEGGED_SDK;

class KeyboardInterface {
   public:
    KeyboardInterface();
    ~KeyboardInterface();
    bool initialize();
    void run();

   private:
    struct termios old_term_settings;
    bool keyboard_active;
    std::shared_ptr<DDSPublisher<ByteArray_>> m_keyboard_pub;
    ByteArray_ keyboard_data;
    XboxJoystickState state;
};

KeyboardInterface::KeyboardInterface() : keyboard_active(false) {
    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &old_term_settings);

    // Configure terminal for non-canonical, non-echo input
    struct termios new_term_settings = old_term_settings;
    new_term_settings.c_lflag &=
        ~(ICANON | ECHO);               // Disable canonical mode and echo
    new_term_settings.c_cc[VMIN] = 0;   // Minimum bytes to return
    new_term_settings.c_cc[VTIME] = 0;  // No timeout
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term_settings);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    keyboard_active = true;
}

KeyboardInterface::~KeyboardInterface() {
    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_term_settings);
}

bool KeyboardInterface::initialize() {
    m_keyboard_pub = std::make_shared<DDSPublisher<ByteArray_>>(
        "rt/keyboard_bytearray/joystick_data", 0);
    if (!m_keyboard_pub) {
        std::cerr << "Failed to initialize DDS publisher" << std::endl;
        return false;
    }
    keyboard_data = ByteArray_();
    keyboard_data.data().resize(sizeof(XboxJoystickState));

    // Set priority
    state.fields.priority = 100;

    // Initialize state to 0
    std::memset(&state, 0, sizeof(state));

    std::cout << "Keyboard interface initialized.\n";
    std::cout << "Controls:\n";
    std::cout << "- Arrow keys: Adjust axes (Left/Right: Left Stick X, "
                 "Up/Down: Left Stick Y)\n";
    std::cout << "- Spacebar: Reset axes to 0\n";
    std::cout << "- Numeric keys (1-9): Press buttons (1:A, 2:B, 3:X, 4:Y, "
                 "5:LB, 6:RB, 7:Back, 8:Start, 9:LS)\n";
    return true;
}

void KeyboardInterface::run() {
    using namespace std::chrono;
    auto next_publish = steady_clock::now();

    while (true) {
        if (keyboard_active) {
            auto now = steady_clock::now();
            auto timeout = next_publish - now;
            if (timeout < 0ms) timeout = 0ms;

            // Check for input
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);

            timeval tv;
            auto us = duration_cast<microseconds>(timeout).count();
            tv.tv_sec = us / 1000000;
            tv.tv_usec = us % 1000000;

            int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
            if (ret < 0) {
                std::cerr << "select error" << std::endl;
                break;
            } else if (ret > 0) {
                char key;
                while (read(STDIN_FILENO, &key, 1) == 1) {
                    if (key == '\033') {
                        // Handle arrow key escape sequences
                        char seq[2];
                        if (read(STDIN_FILENO, &seq[0], 1) == 1 &&
                            read(STDIN_FILENO, &seq[1], 1) == 1) {
                            if (seq[0] == '[') {
                                if (seq[1] == 'A') {  // Up arrow
                                    state.fields.ly +=
                                        3277;  // Approx 0.1 * 32767
                                    if (state.fields.ly > 32767)
                                        state.fields.ly = 32767;
                                } else if (seq[1] == 'B') {  // Down arrow
                                    state.fields.ly -= 3277;
                                    if (state.fields.ly < -32768)
                                        state.fields.ly = -32768;
                                } else if (seq[1] == 'C') {  // Right arrow
                                    state.fields.lx += 3277;
                                    if (state.fields.lx > 32767)
                                        state.fields.lx = 32767;
                                } else if (seq[1] == 'D') {  // Left arrow
                                    state.fields.lx -= 3277;
                                    if (state.fields.lx < -32768)
                                        state.fields.lx = -32768;
                                }
                            }
                        }
                    } else if (key >= '1' && key <= '9') {
                        XboxButton btn = (XboxButton)0;
                        switch (key) {
                            case '1':
                                btn = XBOX_BTN_A;
                                break;
                            case '2':
                                btn = XBOX_BTN_B;
                                break;
                            case '3':
                                btn = XBOX_BTN_X;
                                break;
                            case '4':
                                btn = XBOX_BTN_Y;
                                break;
                            case '5':
                                btn = XBOX_BTN_LB;
                                break;
                            case '6':
                                btn = XBOX_BTN_RB;
                                break;
                            case '7':
                                btn = XBOX_BTN_BACK;
                                break;
                            case '8':
                                btn = XBOX_BTN_START;
                                break;
                            case '9':
                                btn = XBOX_BTN_LS;
                                break;
                        }
                        PRESS_BUTTON(state, btn);
                    } else if (key == ' ') {
                        // Reset axes
                        state.fields.lx = 0;
                        state.fields.ly = 0;
                        state.fields.rx = 0;
                        state.fields.ry = 0;
                        state.fields.lt = 0;
                        state.fields.rt = 0;
                    }
                }
            }

            // Publish data every 20ms
            now = steady_clock::now();
            if (now >= next_publish) {
                memcpy(&keyboard_data.data()[0], state.buffer,
                       sizeof(XboxJoystickState));
                m_keyboard_pub->publish(keyboard_data);

                // Reset buttons after publishing (pulse behavior)
                state.fields.buttons = 0;

                next_publish += milliseconds(20);
                while (next_publish < now) {
                    next_publish += milliseconds(20);  // Catch up if delayed
                }
            }
        }
    }
}

int main() {
    KeyboardInterface keyboard;
    if (!keyboard.initialize()) {
        return 1;
    }
    keyboard.run();
    return 0;
}