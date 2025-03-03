#pragma once

#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"

class Button {
    public:
        Button(const pros::controller_id_e_t id, const pros::controller_digital_e_t button) : m_id(id), m_button(button) {};

        void update() {
            isHeld = pros::c::controller_get_digital(m_id, m_button);
            isPressed = isHeld && !isPrevHeld;
            isReleased = !isHeld && isPrevHeld;
            
            // Rising edge
            if (isPressed) {
                holdStart = from_msec(pros::millis());
            }
            
            // Falling edge
            if (isReleased) {
                lastHoldTime = holdTime; // set last hold time
                holdStart = -1_msec;
            }
            
            prevHoldTime = holdTime;
            holdTime = (isHeld) ? from_msec(pros::millis()) - holdStart : -1_msec;
            isPrevHeld = isHeld;
        }

        [[nodiscard]] bool held() const { return isHeld; }
        [[nodiscard]] bool pressed() const { return isPressed; }
        [[nodiscard]] bool released() const { return isReleased; }

        [[nodiscard]] Time getHoldTime() const { return holdTime; }
        [[nodiscard]] Time getLastHoldTime() const { return lastHoldTime; }
        [[nodiscard]] bool lastHeldFor(Time time) const { return getLastHoldTime() >= time; }
        // Only activates once, for a continuous output compare target with `getHoldTime()`
        [[nodiscard]] bool heldFor(Time time) const { return prevHoldTime < time && getHoldTime() >= time; }

        [[nodiscard]] explicit operator bool() const { return isHeld; }
        [[nodiscard]] explicit operator Time() const { return getHoldTime(); }
    protected:
        bool isPrevHeld = false, isHeld = false, isPressed = false, isReleased = false;
        Time holdStart = 0_msec, holdTime = 0_msec;
        Time lastHoldTime = 0_msec;
        Time prevHoldTime = 0_msec;
        pros::controller_id_e_t m_id;
        pros::controller_digital_e_t m_button;
};

class StickAxis {
    public:
        StickAxis(const pros::controller_id_e_t id, const pros::controller_analog_e_t axis, const int deadzone = 10)
          : m_id(id), m_axis(axis), m_deadzone(deadzone) {};

        [[nodiscard]] int getValue() const {
            int value = pros::c::controller_get_analog(m_id, m_axis);
            return (std::abs(value) > m_deadzone) ? value : 0;
        }
        explicit operator int() const { return getValue(); }
    protected:
        pros::controller_id_e_t m_id;
        pros::controller_analog_e_t m_axis;
        const int m_deadzone;
};

class JoyStick {
    public:
        enum class StickSide { LEFT, RIGHT };

        JoyStick(StickAxis& x, StickAxis& y) : m_x(x), m_y(y) {};
        JoyStick(const pros::controller_id_e_t id, const StickSide side, const int deadzone = 10)
          : m_x({id, (side == StickSide::LEFT) ? pros::E_CONTROLLER_ANALOG_LEFT_X : pros::E_CONTROLLER_ANALOG_RIGHT_X, deadzone}),
            m_y({id, (side == StickSide::LEFT) ? pros::E_CONTROLLER_ANALOG_LEFT_Y : pros::E_CONTROLLER_ANALOG_RIGHT_Y, deadzone}) {};

        [[nodiscard]] int x() const { return m_x.getValue(); }
        [[nodiscard]] int y() const { return m_y.getValue(); }
        
        StickAxis m_x;
        StickAxis m_y;
    protected:
};

class Gamepad {
    public:
        Gamepad(const pros::controller_id_e_t id)
          : controller(pros::Controller(id)),
            l1(id, pros::E_CONTROLLER_DIGITAL_L1),
            l2(id, pros::E_CONTROLLER_DIGITAL_L2),
            r1(id, pros::E_CONTROLLER_DIGITAL_R1),
            r2(id, pros::E_CONTROLLER_DIGITAL_R2),
            d_up(id, pros::E_CONTROLLER_DIGITAL_UP),
            d_down(id, pros::E_CONTROLLER_DIGITAL_DOWN),
            d_left(id, pros::E_CONTROLLER_DIGITAL_LEFT),
            d_right(id, pros::E_CONTROLLER_DIGITAL_RIGHT),
            x(id, pros::E_CONTROLLER_DIGITAL_X),
            b(id, pros::E_CONTROLLER_DIGITAL_B),
            y(id, pros::E_CONTROLLER_DIGITAL_Y),
            a(id, pros::E_CONTROLLER_DIGITAL_A),
            stickLeft(id, JoyStick::StickSide::LEFT),
            stickRight(id, JoyStick::StickSide::RIGHT) {}

        Button l1, l2, r1, r2, d_up, d_down, d_left, d_right, x, b, y, a;
        JoyStick stickLeft, stickRight;

        void update() {
            for (auto button : buttons) {
                button->update();
            }
        }

        Button* operator[] (pros::controller_digital_e_t buttonEnum) {
            return buttons[buttonEnum-6];
        }

        StickAxis* operator[] (pros::controller_analog_e_t axisEnum) {
            return axes[axisEnum];
        }

        pros::Controller controller;
    protected:
        std::array<StickAxis*, 4> axes = {&stickLeft.m_x, &stickLeft.m_y, &stickRight.m_x, &stickRight.m_y};
        std::array<Button*, 12> buttons = {&l1, &l2, &r1, &r2, &d_up, &d_down, &d_left, &d_right, &x, &b, &y, &a};
};