#pragma once

#include <queue>
#include "pros/rtos.hpp"
#include "util.hpp"

class Hooks {
    public:
        Hooks() = delete;

        Hooks(MotorPtr intakeMotor, OpticalPtr optical, int chainLength, std::vector<double> hookPositions)
            : m_motor(std::move(intakeMotor)),
              m_optical(std::move(optical)),
              chainLength(chainLength),
              hooks(hookPositions) {}

        enum class states { FORWARDS, REVERSE, IDLE, UNJAM, INDEX_UP, INDEX_DOWN, WAIT_FOR_RING, COLOUR_SORT };

        /**
         * @brief Get the current state machine state
         *
         * @return states
         */
        [[nodiscard]] states getState() const { return m_state; }

        /**
         * @brief Set the state machine state
         *
         * @param state the state to set the state machine to
         * @param force whether to add to queue or force the change
         */
        void setState(states state, bool force = false) {
            if (force) m_state = state;
            else stateQueue.push(state);
        }

        /**
         * @brief Initialize the hooks
         *
         */
        void initialize();

        /**
         * @brief Check if the hooks are jammed
         *
         * @param countThresh the number of jam states in the last 5 required for a jam to be detected
         * @return true
         * @return false
         */
        [[nodiscard]] bool isJammed(int countThresh = 4) const;

        /**
         * @brief
         *
         * @return Alliance
         */
        [[nodiscard]] Alliance detectRing() const {
            if (m_optical->get_proximity() < detectionProximity) return Alliance::NONE;
            return red.inRange(m_optical->get_hue())    ? Alliance::RED
                   : blue.inRange(m_optical->get_hue()) ? Alliance::BLUE
                                                        : Alliance::NONE;
        };

        /**
         * @brief Sanitize a position to be within [0, chainLength)
         * @param position the position to sanitize
         *
         * @return double the sanitized position
         */
        [[nodiscard]] double sanitizePosition(double position) const { return std::fmod(position, chainLength); }

        /**
         * @brief Get the position of the init hook
         * @param hookNum the hook number to get the position of (index)
         * @return double position in range [0, chainLength)
         */
        [[nodiscard]] double getPosition(int hookNum = 0) const;

        /**
         * @brief Get the vector distance between the target and position
         * 
         * @param target
         * @param position
         * @return double 
         */
        [[nodiscard]] double dist(double target, double position, lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO) const;

        /**
         * @brief Get the id (index) of the nearest hook to a given position (positive = cw, negative = ccw)
         *
         * @param target the target position to find the nearest hook to in range [0, chainLength)
         * @param direction the direction to search in (AUTO = closest direction)
         * @note cw = mogo scoring direction, ccw = outtake/wall scoring direction

         * @return int the id number of the nearest hook
         */
        [[nodiscard]] int
        getNearestHook(double target, lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO) const;

        /**
         * @brief Check if there is a hook at a specific position
         *
         * @param position the position [0, chainLength) to check
         * @param tolerance the acceptable distance for the hook
         * @return bool
         */
        [[nodiscard]] bool isAtPosition(double position, double tolerance = 0.5) const {
            return std::fabs(dist(position, getNearestHook(position))) < tolerance;
        }

        /**
         * @brief Move the nearest hook to a specific position
         * @note cw = mogo scoring direction, ccw = outtake/wall scoring direction
         * @param position the position [0, chainLength) to move the nearest hook to
         * @return double the output of the p loop
         */
        [[nodiscard]] double moveToPosition(double position, int hookNum = -1, lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO);

        /**
         * @brief Update the hooks
         *
         * @param sortAlliance the alliance to sort for
         * @param detectedRing the alliance of the detected ring
         * @param isArmUp whether the arm is up
         */
        void update(Alliance sortAlliance, Alliance detectedRing, bool isArmUp);
    protected:
        const double kP = 0.5;
        const double ringWaitPosition = 0.5;
        const double colourSortPosition = 40;

        std::atomic<bool> isBusy = false;
        states m_state = states::IDLE, lastState = states::IDLE, prevState = states::IDLE;
        std::queue<states> stateQueue;

        /**
         * @brief Get the next state in the queue
         *
         * @return states
         */
        states nextState() {
            if (stateQueue.empty()) return states::IDLE;
            states next = stateQueue.front();
            stateQueue.pop();
            return next;
        }

        MotorPtr m_motor = nullptr;
        int currentMotorVoltage = 0, prevMotorVoltage = 0, prevStateVoltage = 0;

        void setVoltage(int voltage) { currentMotorVoltage = voltage; }

        OpticalPtr m_optical = nullptr;
        int detectionProximity = 100;

        int chainLength;

        std::vector<double> hooks;
        std::vector<bool> jamDetects;
        const std::pair<int, double> jamThresh = {1500, 5};
};

using HooksPtr = std::unique_ptr<Hooks>;