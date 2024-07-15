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

Conveyor::Conveyor(Intake& intake, Hooks& hooks)
    : intake(intake),
      hooks(hooks){}

void Conveyor::update() {
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
        case Conveyor::state::UNJAM: {
            if (this->prevState == Conveyor::state::FORWARDS) { this->hooks.move(-50); }
            else { this->hooks.move(50); }

            if (!this->hooks.isJammed()) { this->currState = Conveyor::state::IDLE; }

            break;
        }
        case Conveyor::state::INDEX: {
            this->hooks.move(-127);
            break;
        }
        case Conveyor::state::IDLE: {
            this->intake.move(0);
            this->hooks.move(0);
            break;
        }
    }
}

void Conveyor::forwards() {
    if (this->currState != Conveyor::state::UNJAM) { this->currState = Conveyor::state::FORWARDS; }
}

void Conveyor::reverse() {
    if (this->currState != Conveyor::state::UNJAM) { this->currState = Conveyor::state::REVERSE; }
}

void Conveyor::idle() {
    if (this->currState != Conveyor::state::UNJAM) { this->currState = Conveyor::state::IDLE; }
}

void Conveyor::unjam() {
    this->currState = Conveyor::state::UNJAM;
}

void Conveyor::queueIndex() {
    this->indexQueue.push(this->getIndexPose());
}

double Conveyor::getIndexPose() {
    return 0;
}


