#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "robot/chassis/chassis.hpp"

void CustomChassis::swagTurnToHeading(float theta, DriveSide startLockSide, int timeout, TurnToHeadingParams params,
                                      bool async) {
    params.minSpeed = std::fabs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { swagTurnToHeading(theta, startLockSide, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta = sanitizeAngle(180 - theta);
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    if (params.direction == AngularDirection::AUTO) {
        if (angleError(targetTheta, startTheta, false, AngularDirection::CW_CLOCKWISE) >
            angleError(targetTheta, startTheta, false, AngularDirection::CCW_COUNTERCLOCKWISE))
            params.direction = AngularDirection::CCW_COUNTERCLOCKWISE;
    }
    DriveSide endLockSide = (startLockSide == DriveSide::LEFT) ? DriveSide::RIGHT : DriveSide::LEFT;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // get original braking mode of that side of the drivetrain so we can set it back to it after this motion ends
    pros::MotorBrake brakeMode = this->drivetrain.leftMotors->get_brake_mode(0);

    // find thresholds for each turn stage
    int dirMultiplier = (params.direction == AngularDirection::CW_CLOCKWISE) ? 1 : -1;
    double firstSwingEnd = startTheta + 60 * dirMultiplier;
    double secondSwingStart = targetTheta - 60 * dirMultiplier;
    if (dirMultiplier * angleError(secondSwingStart, firstSwingEnd) < 0) {
        firstSwingEnd = (startTheta + targetTheta) / 2;
        secondSwingStart = firstSwingEnd;
    }

    enum class TurnStage {
        START,
        MIDDLE,
        END,
    };

    auto getTurnStage = [&](double currentTheta) -> TurnStage {
        if (dirMultiplier * angleError(firstSwingEnd, currentTheta, false, params.direction) < 0) {
            return TurnStage::START;
        } else if (dirMultiplier * angleError(secondSwingStart, currentTheta, false, params.direction) < 0) {
            return TurnStage::MIDDLE;
        } else {
            return TurnStage::END;
        }
    };

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();
        pose.theta = fmod(pose.theta, 360);

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));
        targetTheta = sanitizeAngle(180 - theta);

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        if (getTurnStage(pose.theta) != TurnStage::MIDDLE) { motorPower *= 2; }

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        // No access to infosink sadge

        // move the drivetrain
        auto motorBrake = [&](pros::MotorGroup* motorGroup) {
            motorGroup->set_brake_mode_all(pros::MotorBrake::hold);
            motorGroup->brake();
        };

        auto motorMove = [&](pros::MotorGroup* motorGroup, float power) {
            motorGroup->set_brake_mode_all(brakeMode);
            motorGroup->move(power);
        };

        if (getTurnStage(pose.theta) == TurnStage::START) {
            if (startLockSide == DriveSide::LEFT) {
                motorBrake(drivetrain.leftMotors);
                motorMove(drivetrain.rightMotors, motorPower);
            } else {
                motorMove(drivetrain.leftMotors, -motorPower);
                motorBrake(drivetrain.rightMotors);
            }
        } else if (getTurnStage(pose.theta) == TurnStage::MIDDLE) {
            motorMove(drivetrain.leftMotors, motorPower);
            motorMove(drivetrain.rightMotors, -motorPower);
        } else {
            if (endLockSide == DriveSide::LEFT) {
                motorBrake(drivetrain.leftMotors);
                motorMove(drivetrain.rightMotors, -motorPower);
            } else {
                motorMove(drivetrain.leftMotors, motorPower);
                motorBrake(drivetrain.rightMotors);
            }
        }

        // delay to save resources
        pros::delay(10);
    }
    // reset all brake modes
    drivetrain.leftMotors->set_brake_mode_all(brakeMode);
    drivetrain.rightMotors->set_brake_mode_all(brakeMode);
    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}