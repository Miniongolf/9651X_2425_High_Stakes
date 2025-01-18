#pragma once

#include "robot/constants.hpp"
#include "util.hpp"


class Clamp {
    public:
        /**
         * @brief Construct a new mogo clamp object
         * @param piston The piston that clamps
         * @param sensor The distance sensor for autoclamp
         * @param chassis The lemlib chassis to adjust PID gains for
         */
        Clamp(PistonPtr piston, DistancePtr sensor, lemlib::Chassis* chassis) : m_piston(std::move(piston)), m_sensor(std::move(sensor)), m_chassis(chassis) {};

        void initialize() {
            reset();
            pros::Task clampTask([&](){taskFunct();});
        }

        void clamp() {
            m_piston->extend();
            m_chassis->lateralPID.setGains(mogoLateralPID);
            m_chassis->angularPID.setGains(mogoAngularPID);
            cancelAutoClamp();
        }

        void release() {
            m_piston->retract();
            m_chassis->lateralPID.setGains(emptyLateralPID);
            m_chassis->angularPID.setGains(emptyAngularPID);
            cancelAutoClamp();
        }

        void setState(bool isClamped) {
            (isClamped) ? clamp() : release();
        }

        void toggle() {
            setState(!m_piston->is_extended());
        }

        Length getDistance() {
            return from_mm(m_sensor->get());
        }

        bool seesMogo() {
            if (getDistance() > longDist) {
                return false;
            }
            int longCount = 0, shortCount = 0;
            for (Length dist : distQueue) {
                if (dist < longDist) {
                    return true;
                }
            }
            return longCount >= distQueue.size() / 2 && shortCount >= 2;
        }

        /**
         * @brief Request the clamp to automatically clamp when it sees a mogo
         * @note Opens the clamp if it is closed at this point
         */
        void requestAutoClamp() {
            isAutoClamping = true;
            release();
        }

        /**
         * @brief Cancel the auto clamp request
         * @note Does not force open the clamp
         */
        void cancelAutoClamp() {
            isAutoClamping = false;
        }

        void reset() {
            release();
            cancelAutoClamp();
        }
    protected:
        void taskFunct() {
            pros::delay(10);
            distQueue.push_back(getDistance());
            if (distQueue.size() > 10) {
                distQueue.erase(distQueue.begin());
            }
            if (isAutoClamping && seesMogo()) { clamp(); }
        }
        PistonPtr m_piston = nullptr;
        DistancePtr m_sensor = nullptr;
        lemlib::Chassis* m_chassis = nullptr;

        static constexpr Length longDist = 750_mm, shortDist = 250_mm;
        std::vector<Length> distQueue;

        bool isAutoClamping = false;
};