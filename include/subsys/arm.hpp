#pragma once

#include <memory>
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"

class Arm {
    public:
        Arm(std::unique_ptr<pros::Motor> leftMotor,
            std::unique_ptr<pros::Rotation> leftEnc, double leftRatio,
            std::unique_ptr<pros::Motor> rightMotor,
            std::unique_ptr<pros::Rotation> rightEnc, double rightRatio,
            lemlib::PID pid, int rpm);
        ~Arm() {this->task.remove();};

        enum class state {
            MOVING,
            HOLD,
            DESCORE,
            STOP,
            INACTIVE
        };

        double angleOffset = 0;
//        bool autonMovement = true;

        void reset();

        static double angleToHeight(double angle) { return 18.5 + 12.5*std::sin(angle); }
        static double heightToAngle(double height) { return std::asin((height - 18.5) / 12.5); }

        double getAngle() { return (this->getLeftAngle() + this->getRightAngle())/2; }
        double getHeight() { return angleToHeight(this->getAngle()); }
        void moveToAngle(double angle);
        void moveToHeight(double height);
        void changeAngle(double deltaAngle);
        void changeHeight(double deltaHeight);
        void disconnect();
        void connect();

        void descore();

        void resumeTask() {this->task.resume();};
        void suspendTask() {this->task.suspend();};

        double getLeftAngle();
        double getRightAngle();
        bool isInPosition() { return this->currState == Arm::state::HOLD; }
    private:

        std::unique_ptr<pros::Motor> leftMotor, rightMotor;
        std::unique_ptr<pros::Rotation> leftRot, rightRot;
        double leftRatio, rightRatio;

        lemlib::PID leftPID, rightPID;
        int rpm;
        double targetAngle = -50;

        Arm::state currState = Arm::state::INACTIVE;

        pros::Task task = pros::Task {[&] {
            while (true) {
                pros::delay(10);
                double error = lemlib::angleError(this->getAngle(), this->targetAngle, false);
                double leftError = lemlib::angleError(this->getLeftAngle(), this->targetAngle + angleOffset, false);
                double rightError = lemlib::angleError(this->getRightAngle(), this->targetAngle - angleOffset, false);

//                std::printf("Arm: %f | %f\n", this->getAngle(), this->targetAngle);

                if (this->currState == Arm::state::INACTIVE) continue;

                if (std::fabs(leftError) <= 4 && std::fabs(rightError) <= 4) {
                    this->currState = Arm::state::HOLD;
                } else {
                    this->currState = Arm::state::MOVING;
                }

                if (this->currState == Arm::state::MOVING) {
                    double leftVel = this->leftPID.update(leftError), rightVel = this->rightPID.update(rightError);
                    this->leftMotor->move(leftVel);
                    this->rightMotor->move(rightVel);
                } else if (this->currState == Arm::state::HOLD) {
                    this->leftMotor->move(0);
                    this->rightMotor->move(0);
                }
            }
        }};;
};
