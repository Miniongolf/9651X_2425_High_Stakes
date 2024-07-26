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
      optical(std::move(optical)){}

void Conveyor::update() {
    this->isBusy = (this->currState == Conveyor::state::UNJAM) || (this->currState == Conveyor::state::INDEX);

    if (this->hooks.isJammed()) this->unjam();

    if (this->currState != this->prevState) {
        this->prevState = this->currState;
    }

    switch (this->currState) {
        case Conveyor::state::FORWARDS: {
            this->intake.move(127);
            this->hooks.move(127);
            break;
        }
        case Conveyor::state::REVERSE: {
            this->intake.move(-127);
            this->hooks.move(-70);
            break;
        }
        case Conveyor::state::INDEX: {
            break;
        }
        case Conveyor::state::STOP: {
            this->intake.move(0);
            this->hooks.move(0);
            break;
        }
        case Conveyor::state::IDLE: {break;}
    }
}

void Conveyor::forwards() {
    if (this->isBusy) return;
    this->currState = Conveyor::state::FORWARDS;
}

void Conveyor::reverse() {
    if (this->isBusy) return;
    this->currState = Conveyor::state::REVERSE;
}

void Conveyor::stop() {
    if (this->isBusy) return;
    this->currState = Conveyor::state::STOP;
}

void Conveyor::unjam() {
    if (this->isBusy) return;
    this->currState = Conveyor::state::UNJAM;
}

void Conveyor::queueIndex() {
    this->indexQueue += 1;
    this->index();
}

void Conveyor::idle() {
    this->currState = Conveyor::state::IDLE;
}

double Conveyor::getIndexPose() {
    double zeroDistance = lemlib::angleError(this->hooks.getPose(), 0, false, lemlib::AngularDirection::CW_CLOCKWISE);
    double turnDistance = lemlib::angleError(this->hooks.getPose(), 180, false, lemlib::AngularDirection::CW_CLOCKWISE);
    return zeroDistance < turnDistance ? 0 : 180;
}

void Conveyor::index() {
    if (this->indexQueue == 0) { return; }
    this->currState = Conveyor::state::INDEX;
}


