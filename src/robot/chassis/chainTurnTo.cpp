#include "lemlib/chassis/chassis.hpp"
#include "robot/chassis/chassis.hpp"

void CustomChassis::chainTurnToHeading(float theta, bool startForwards, bool endForwards, int timeout, ChainTurnParams params, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { chainTurnToHeading(theta, startForwards, endForwards, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    bool isLongTurn = startForwards != endForwards; // Turn the long way if the bot flips directions
    float startTheta = getPose().theta;
    float targetTheta = theta;
    float shortDeltaTheta = angleError(targetTheta, startTheta, false, AngularDirection::AUTO);
    AngularDirection longDirection =
        (shortDeltaTheta > 0) ? AngularDirection::CCW_COUNTERCLOCKWISE : AngularDirection::CW_CLOCKWISE;
    AngularDirection shortDirection = (longDirection == AngularDirection::CW_CLOCKWISE)
                                          ? AngularDirection::CCW_COUNTERCLOCKWISE
                                          : AngularDirection::CW_CLOCKWISE;
    AngularDirection turnDirection = (isLongTurn) ? longDirection : shortDirection;
    int dirMultip = (turnDirection == AngularDirection::CW_CLOCKWISE) ? 1 : -1;

    float motorPower;
    float prevMotorPower = 0;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // get original braking mode of that side of the drivetrain so we can set it back to it after this motion ends
    pros::MotorBrake stdBrakeMode = this->drivetrain.leftMotors->get_brake_mode(0);
    // xor to find which side to lock at the start and end for the swing turn
    bool startLockRight = (startForwards != (turnDirection == AngularDirection::CCW_COUNTERCLOCKWISE)) ? true : false;
    DriveSide startLockSide = (startLockRight) ? DriveSide::RIGHT : DriveSide::LEFT;
    bool endLockRight = (endForwards != (turnDirection == AngularDirection::CCW_COUNTERCLOCKWISE)) ? true : false;
    DriveSide endLockSide = (endLockRight) ? DriveSide::RIGHT : DriveSide::LEFT;

    // find thresholds for each turn stage
    double firstSwingEnd = startTheta + 45 * dirMultip;
    double secondSwingStart = targetTheta - 45 * dirMultip;

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();
        pose.theta = fmod(pose.theta, 360);
        float deltaTheta = angleError(targetTheta, pose.theta, false, turnDirection);

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));
        targetTheta = theta;

        // ignore settling (this function is only for chaining)
        // exit once at target (within 1º or past target)
        if (fabs(deltaTheta) < 1 || sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        infoSink()->debug("Turn Motor Power: {} ", motorPower);

        // move the drivetrain
        auto motorBrake = [&](pros::MotorGroup* motorGroup) {
            motorGroup->set_brake_mode_all(pros::MotorBrake::hold);
            motorGroup->brake();
        };
        auto motorMove = [&](pros::MotorGroup* motorGroup, float power) {
            motorGroup->set_brake_mode_all(stdBrakeMode);
            motorGroup->move(power);
        };

        if (fabs(angleError(pose.theta, startTheta)) < params.swingAngle) {
            if (startLockSide == DriveSide::LEFT) {
                motorBrake(drivetrain.leftMotors);
                motorMove(drivetrain.rightMotors, -motorPower);
            } else {
                motorMove(drivetrain.leftMotors, motorPower);
                motorBrake(drivetrain.rightMotors);
            }
        } else if (fabs(angleError(targetTheta, pose.theta)) < params.swingAngle) {
            if (endLockSide == DriveSide::LEFT) {
                motorBrake(drivetrain.leftMotors);
                motorMove(drivetrain.rightMotors, -motorPower);
            } else {
                motorMove(drivetrain.leftMotors, motorPower);
                motorBrake(drivetrain.rightMotors);
            }
        } else {
            motorMove(drivetrain.leftMotors, motorPower);
            motorMove(drivetrain.rightMotors, -motorPower);
        }
    }
}