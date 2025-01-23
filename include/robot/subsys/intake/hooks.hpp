#pragma once

#include "util.hpp"

class Hooks {
    public:
        Hooks(MotorPtr intakeMotor, OpticalPtr optical, int chainLength, std::vector<double> hookPositions)
            : m_motor(std::move(intakeMotor)),
              m_optical(std::move(optical)),
              chainLength(chainLength),
              hooks(hookPositions) {
            m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            m_motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
        }

        enum class states { FORWARDS, REVERSE, IDLE, WAIT_FOR_RING, MOVE };

        states getState() const { return currState; }

        void setState(states state, bool forceInstant = false, bool clearQueue = false);

        void forwards() { setState(states::FORWARDS); }

        void reverse() { setState(states::REVERSE); }

        void idle() { setState(states::IDLE); }

        void initialize() { m_motor->tare_position(); }

        [[nodiscard]] double sanitizePosition(double position) const { return std::fmod(position, chainLength); }

        [[nodiscard]] double getPosition(int hookNum = 0) const;

        [[nodiscard]] double dist(double target, double position,
                                  lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO) const;

        [[nodiscard]] int getNearestHook(double target,
                                         lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO) const;

        [[nodiscard]] bool isAtPosition(double target, int hookNum = -1, double tolerance = 0.5) const;

        [[nodiscard]] double moveToPosition(double position, int hookNum = -1,
                                            lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO);

        [[nodiscard]] bool isJammed() const;

        void update(bool hasPrerollRing);

        friend std::ostream& operator<<(std::ostream& os, const Hooks& hooks) {
            os << "Hooks pose (" << hooks.getPosition(0) << ", " << hooks.getPosition(1) << ", " << hooks.getPosition(2) << ", "
               << hooks.getPosition(3) << ") --> " << hooks.getNearestHook(hooks.idlePose) << "\n";
            return os;
        }
    protected:
        // Devices
        MotorPtr m_motor = nullptr;
        OpticalPtr m_optical = nullptr;

        // Hook positions
        const int chainLength;
        const std::vector<double> hooks;
        const double idlePose = 0, colourSortPose = 40;

        // State machines
        states currState = states::IDLE, lastState = states::IDLE, prevState = states::IDLE;
        std::deque<states> stateQueue;
        void nextState();

        // Voltage utils
        int currVoltage = 0, prevVoltage = 0;

        void setVoltage(int voltage) { currVoltage = voltage; }

        lemlib::PID pid = {20, 0, 0};

        // Jam detection
        std::vector<bool> jamDetects = std::vector<bool>(5, false); // This construction returns all falses
        const std::pair<int, double> jamThresh = {900, 5};
};

using HooksPtr = std::unique_ptr<Hooks>;