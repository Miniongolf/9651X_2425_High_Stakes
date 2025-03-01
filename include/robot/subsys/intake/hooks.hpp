#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "util.hpp"
#include <algorithm>

class Hooks {
    public:
        Hooks(MotorPtr intakeMotor, OpticalPtr optical, RotationPtr rotSensor, int chainLength,
              std::vector<double> hookPositions)
            : m_motor(std::move(intakeMotor)),
              m_optical(std::move(optical)),
              m_rotSensor(std::move(rotSensor)),
              chainLength(chainLength),
              hooks(hookPositions) {
            m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            m_motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
        }

        // Init
        void initialize(Alliance alliance) {
            std::printf("hooks init\n");
            m_optical->set_integration_time(8);
            m_optical->set_led_pwm(100);
            m_motor->tare_position();
            m_rotSensor->set_position(0);
            m_alliance = alliance;
        }

        // State machine
        enum class states { FORWARDS, REVERSE, IDLE, INDEX };

        states getState() const { return currState; }

        void setState(states state, bool forceInstant = true, bool clearQueue = true);

        bool atState(states state) const { return currState == state; }

        bool atState(std::vector<states> stateList) const {
            return std::find(stateList.begin(), stateList.end(), currState) != stateList.end();
        }

        bool busy() const { return isBusy; }

        void forwards() { setState(states::FORWARDS); }

        void reverse() { setState(states::REVERSE); }

        void idle() { setState(states::IDLE); }

        void update(bool hasPrerollRing, bool forcedIndex, bool isArmDown, bool isArmUp, bool isArmStuck);

        // Position utils
        [[nodiscard]] double sanitizePosition(double position) const { return std::fmod(position, chainLength); }

        [[nodiscard]] double dist(double target, double position,
                                  lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO) const;
        [[nodiscard]] double getPosition(int hookNum = 0) const;
        [[nodiscard]] int getNearestHook(double target,
                                         lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO, double tolerance = 3) const;
        [[nodiscard]] bool isAtPosition(double target, int hookNum = -1, double tolerance = 1) const;
        int poseOffset = 0;

        // Antijam
        [[nodiscard]] bool isJammed() const;

        // Colour sort
        [[nodiscard]] Alliance ringDetect() const;
        bool colourSortEnabled = true;
        Alliance m_alliance = Alliance::RED;

        // Telemetry
        [[nodiscard]] double get_temperature() const { return m_motor->get_temperature(); }
        friend std::ostream& operator<<(std::ostream& os, const Hooks& hooks) {
            os << "Hooks pose (" << hooks.getPosition(0) << ", " << hooks.getPosition(1) << ", " << hooks.getPosition(2)
               << ", " << hooks.getPosition(3) << ") --> " << hooks.getNearestHook(hooks.idlePose) << "\n";
            return os;
        }

    protected:
        // Devices
        MotorPtr m_motor = nullptr;
        OpticalPtr m_optical = nullptr;
        RotationPtr m_rotSensor = nullptr;

        // State machine
        states currState = states::IDLE, lastState = states::IDLE, prevState = states::IDLE;
        std::deque<states> stateQueue;
        void nextState();
        bool isBusy = false;

        // Hook positions
        const int chainLength;
        const std::vector<double> hooks;
        const double idlePose = 0, colourSortPose = 2;
        void moveTowards(double target, int hookNum, lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO, double settleRange = 3);

        // Colour sort
        const int proxRange = 125; // Tune this
        int colourDetectHook = 0;

        // Hold mode flags
        bool sawPrerollRing = false;
        bool prevHasPrerollRing = false;
        int indexHook = 0;
        lemlib::Timer moveTimer = {0};

        // Voltage utils
        int currVoltage = 0, prevVoltage = 0;
        bool prevIsArmStuck = false;

        void setVoltage(int voltage) { currVoltage = voltage; }

        lemlib::PID pid = {7, 0, 0};

        // Jam detection
        std::vector<bool> jamDetects = std::vector<bool>(5, false); // This construction returns all falses
        const std::pair<int, double> jamThresh = {900, 5}; // {current, velocity}
};

using HooksPtr = std::unique_ptr<Hooks>;