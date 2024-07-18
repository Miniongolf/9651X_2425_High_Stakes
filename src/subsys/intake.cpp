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
      hooks(hooks){
    pros::Task conveyorTask{[&] {
        while (true) {
            if (this->currState == Conveyor::state::UNJAM) {}
            else if (this->currState == Conveyor::state::INDEX) {
                double error = lemlib::angleError(this->hooks.getPose(), this->indexQueue.front(), false, lemlib::AngularDirection::CW_CLOCKWISE);
                std::printf("%f, %f\n", this->indexQueue.front(), error);
                if (error >= this->farIndexThresh) {
                    int hooksVel = this->hooks.farPID.update(error);
                    hooks.move(hooksVel);
                } else if (error <= this->closeIndexThresh) {
                    int hooksVel = std::clamp(this->hooks.closePID.update(error), (float)-35.0, (float)35.0);
                    hooks.move(hooksVel);
                } else {
                    this->hooks.move(-127);
                    this->indexQueue.pop();
                    pros::delay(500);
                    this->idle();
                }
            }
            pros::delay(10);
        }
    }};

    conveyorTask;
}

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
        case Conveyor::state::UNJAM: {
            if (this->prevState == Conveyor::state::FORWARDS) { this->hooks.move(-50); }
            else { this->hooks.move(50); }

            if (!this->hooks.isJammed()) { this->currState = Conveyor::state::IDLE; }

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
    this->indexQueue.push(this->getIndexPose());
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
    if (this->indexQueue.empty()) { return; }
    this->currState = Conveyor::state::INDEX;
}


