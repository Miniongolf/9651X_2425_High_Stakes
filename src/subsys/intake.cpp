#include "subsys/intake.hpp"

Intake::Intake(std::unique_ptr<pros::Motor> intakeMotor)
    : motor(std::move(intakeMotor)) {
    this->motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    this->motor->tare_position();
}

Hooks::Hooks(std::unique_ptr<pros::Motor> hooksMotor)
    : motor(std::move(hooksMotor)) {
    this->motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    this->motor->tare_position();
}

Conveyor::Conveyor(Intake& intake, Hooks& hooks, std::unique_ptr<pros::Optical> optical)
    : intake(intake),
      hooks(hooks),
      optical(std::move(optical)) {
    this->optical->set_led_pwm(50);
}

void Conveyor::update() {
    if (this->hooks.isJammed()) {
        this->unjam();
        std::printf("--! HOOKS JAMMED %f!--\n", this->hooks.getPose());
    } else if (this->detectRing() && this->indexQueue > 0 && !isBusy()) {
        this->index();
        std::printf("--? RING DETECTED | INDEXING ?--\n");
    }

    if (this->currState != this->prevState) {
        this->prevState = this->currState;
    }

    switch (this->currState) {
        case state::FORWARDS: {
            this->intake.move(127);
            this->hooks.move(127);
            break;
        }
        case state::MAN_INDEX: {
            this->intake.move(127);
            this->hooks.move(50);
            break;
        }
        case state::REVERSE: {
            this->intake.move(-127);
            this->hooks.move(-70);
            break;
        }
        case state::INDEX: {
            break;
        }
        case state::STOP: {
            this->intake.move(0);
            this->hooks.move(0);
            break;
        }
        case state::IDLE: {break;}
    }
}

void Conveyor::forwards() {
    if (this->isBusy()) return;
    this->currState = Conveyor::state::FORWARDS;
}

void Conveyor::manualInd() {
    if (this->isBusy()) return;
    this->currState = Conveyor::state::MAN_INDEX;
}

void Conveyor::reverse() {
    if (this->isBusy()) return;
    this->currState = Conveyor::state::REVERSE;
}

void Conveyor::stop() {
    if (this->isBusy()) return;
    this->currState = Conveyor::state::STOP;
}

void Conveyor::unjam() {
    this->currState = Conveyor::state::UNJAM;
}

void Conveyor::queueIndex(int count) {
    if (count < 0) return;
    this->indexQueue += count;
}

void Conveyor::idle() {
    this->currState = Conveyor::state::IDLE;
}

double Conveyor::getIndexPose() {
    double zeroDistance = -lemlib::angleError(this->hooks.getPose(), 0, false);
    double turnDistance = -lemlib::angleError(this->hooks.getPose(), 180, false);
    return (zeroDistance < turnDistance) ? 0 : 180;
}

void Conveyor::index() {
    if (this->indexQueue == 0 || this->isBusy()) { return; }
    this->targetIndexPose = this->getIndexPose();
    this->currState = Conveyor::state::INDEX;
}

void Conveyor::moveToIndex() {
    lemlib::Timer indexTimer(5000);
    hooks.move(50);
    while (this->detectRing() && !indexTimer.isDone()) {
        pros::delay(10);
    }
    hooks.move(-127);
    pros::delay(1000);

//    double error = -lemlib::angleError(this->targetIndexPose, this->hooks.getPose(), false);
////    std::printf("Conveyor Error: %f -> %f\n", this->hooks.getPose(), error);
//    double hooksFF = (error > 0) ? 25 : -25;
//    if (std::fabs(error) <= this->closeIndexThresh) {
//        std::printf("DONE!!!\n");
//        this->hooks.move(-127);
//        this->hooks.closePID.reset();
//        this->hooks.farPID.reset();
//        this->indexQueue -= 1;
//        pros::delay(1000);
//        this->idle();
//
//    } else if (std::fabs(error) <= this->farIndexThresh) {
//        int hooksVel = std::clamp(this->hooks.closePID.update(error), (float)-25.0, (float)25.0);
//        hooks.move(hooksVel + hooksFF);
////        std::printf("Close: %d\n", hooksVel);
//    } else {
//        int hooksVel = this->hooks.farPID.update(error);
//        hooks.move(hooksVel + hooksFF);
////        std::printf("Far: %d\n", hooksVel);
//    }
}