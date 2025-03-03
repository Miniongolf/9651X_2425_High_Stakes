#pragma once

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

        void reset() {
            release();
        }

        void initialize() {
            reset();
            std::printf("mogo clamp init\n");
            pros::Task clampTask([&](){ taskFunct(); });
        }

        bool isClamped() {
            return m_piston->is_extended();
        }

        void clamp(bool wait = false) {
            m_piston->extend();
            m_chassis->lateralPID.setGains(mogoLateralPID);
            m_chassis->angularPID.setGains(mogoAngularPID);
            cancelAutoClamp();
            if (wait) { pros::delay(30); }
        }

        void release(bool wait = false) {
            m_piston->retract();
            m_chassis->lateralPID.setGains(emptyLateralPID);
            m_chassis->angularPID.setGains(emptyAngularPID);
            cancelAutoClamp();
            if (wait) { pros::delay(30); }
        }

        void setState(bool isClamped) {
            (isClamped) ? clamp() : release();
        }

        void toggle() {
            setState(!isClamped());
        }

        Length getDistance() {
            return from_mm(m_sensor->get());
        }

        bool seesMogo() {
            if (distQueue.back() > longDist) {
                return false;
            }
            int longCount = 0, shortCount = 0;
            for (Length& dist : distQueue) {
                longCount += (dist < longDist);
                shortCount += (dist < shortDist);
            }
            return longCount >= std::min((int)distQueue.size(), 5) && shortCount >= std::min((int)distQueue.size(), 2);
        }

        /**
         * @brief Request the clamp to automatically clamp when it sees a mogo
         * @param openClamp Opens the clamp if it is closed at this point
         */
        void requestAutoClamp(bool openClamp = true) {
            if (openClamp) { release(); }
            isAutoClamping = true;
        }

        /**
         * @brief Cancel the auto clamp request
         * @note Does not force open the clamp
         */
        void cancelAutoClamp() {
            isAutoClamping = false;
        }

        bool isAuto() {
            return isAutoClamping;
        }
    protected:
        void taskFunct() {
            int counter = 0;
            while (true) {
                pros::delay(10);
                distQueue.push_back(getDistance());
                if (distQueue.size() > 10) {
                    distQueue.erase(distQueue.begin());
                }
                if (isAutoClamping && seesMogo()) { clamp(); }

                if (counter % 10 == 0) {
                    // std::cout << "mogo dists: " << distQueue[0] << distQueue[1] << distQueue[2] << distQueue[3] << distQueue[4] << distQueue[5] << distQueue[6] << distQueue[7] << distQueue[8] << distQueue[9] << '\n';
                }
                counter++;
            }
        }
        PistonPtr m_piston = nullptr;
        DistancePtr m_sensor = nullptr;
        lemlib::Chassis* m_chassis = nullptr;

        static constexpr Length longDist = 120_mm, shortDist = 60_mm;
        std::vector<Length> distQueue;

        bool isAutoClamping = false;
};