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
    this->optical->set_led_pwm(100);
}

void Conveyor::update() {
    if (this->hooks.isJammed()) {
        this->unjam();
        std::printf("--! HOOKS JAMMED %f!--\n", this->hooks.getPose());
    } else if (this->detectRing() && this->indexQueue > 0 && !isBusy() && this->canIndex) {
        this->index();
        std::printf("--? RING DETECTED | INDEXING ?--\n");
    }

    if (this->currState != this->prevState) {
        this->prevState = this->currState;
    }

    switch (this->currState) {
        case state::FORWARDS: {
            this->intake.move(127);

            if (this->indexQueue > 0) {
                this->hooks.move(90);
            } else {
                this->hooks.move(127);
            }
            this->isReversing = false;
            break;
        }
        case state::MAN_INDEX: {
            this->intake.move(127);
            this->hooks.move(50);
            this->isReversing = false;
            break;
        }
        case state::REVERSE: {
            this->intake.move(-127);
            this->hooks.move(-70);
            this->isReversing = true;
            break;
        }
        case state::INDEX: {
            break;
        }
        case state::STOP: {
            double error0 = -lemlib::angleError(0, this->hooks.getPose(), false);
            double error180 = -lemlib::angleError(180, this->hooks.getPose(), false);

            this->intake.move(0);

            if (std::fabs(error0) < 30 || std::fabs(error180) < 30) {
                this->hooks.move(50);
            } else {
                this->hooks.move(0);
            }
            this->isReversing = false;
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
    indexTimer.resume();

    this->intake.move(-127);
    this->hooks.move(60);
    this->isReversing = false;
    while (this->detectRing() && !indexTimer.isDone()) {
        pros::delay(10);
    }
    pros::delay(70);
    hooks.move(-127);
    this->isReversing = true;
    pros::delay(1500);
    indexQueue -= 1;
}