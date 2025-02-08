#pragma once

#include "util.hpp"
#include <algorithm>

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

        // Init
        void initialize() {
            std::printf("hooks init\n");
            m_motor->tare_position();
        }

        // State machine
        enum class states { FORWARDS, REVERSE, IDLE, WAIT_FOR_RING, MOVE };

        states getState() const { return currState; }
        void setState(states state, bool forceInstant = false, bool clearQueue = false);
        bool atState(states state) const { return currState == state; }
        bool atState(std::vector<states> stateList) const { return std::find(stateList.begin(), stateList.end(), currState) != stateList.end(); }

        bool busy() const { return isBusy; }

        void forwards() { setState(states::FORWARDS); }
        void reverse() { setState(states::REVERSE); }
        void idle() { setState(states::IDLE); }

        void update(bool hasPrerollRing, bool forcedIndex, bool isArmUp);

        // Position utils
        [[nodiscard]] double sanitizePosition(double position) const { return std::fmod(position, chainLength); }
        [[nodiscard]] double dist(double target, double position,
                                  lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO) const;
        [[nodiscard]] double getPosition(int hookNum = 0) const;
        [[nodiscard]] int getNearestHook(double target,
                                         lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO) const;
        [[nodiscard]] bool isAtPosition(double target, int hookNum = -1, double tolerance = 1) const;
        int poseOffset = 0;

        // Antijam
        [[nodiscard]] bool isJammed() const;

        // Colour sort
        [[nodiscard]] Alliance ringDetect() const;
        bool colourSortEnabled = true;

        // Telemetry
        friend std::ostream& operator<<(std::ostream& os, const Hooks& hooks) {
            os << "Hooks pose (" << hooks.getPosition(0) << ", " << hooks.getPosition(1) << ", " << hooks.getPosition(2) << ", "
               << hooks.getPosition(3) << ") --> " << hooks.getNearestHook(hooks.idlePose) << "\n";
            return os;
        }
        
    protected:
        // Devices
        MotorPtr m_motor = nullptr;
        OpticalPtr m_optical = nullptr;

        // State machine
        states currState = states::IDLE, lastState = states::IDLE, prevState = states::IDLE;
        std::deque<states> stateQueue;
        void nextState();
        bool isBusy = false;
        bool ringWaitFlag = false;

        // Hook positions
        const int chainLength;
        const std::vector<double> hooks;
        const double idlePose = 0, colourSortPose = 40;
        void moveTowards(double target, int hookNum, lemlib::AngularDirection direction, double settleRange = 3);

        // Colour sort
        const int proxRange = 100; // Tune this
        bool colourSorting = false;
        int colourDetectHook = 0;

        // Hold mode flags
        bool sawPrerollRing = false;

        // Voltage utils
        int currVoltage = 0, prevVoltage = 0;
        void setVoltage(int voltage) { currVoltage = voltage; }
        lemlib::PID pid = {10, 0, 0};

        // Jam detection
        std::vector<bool> jamDetects = std::vector<bool>(5, false); // This construction returns all falses
        const std::pair<int, double> jamThresh = {900, 5}; // {current, velocity}
};

using HooksPtr = std::unique_ptr<Hooks>;