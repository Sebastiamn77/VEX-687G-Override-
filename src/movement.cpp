#include "override.h"
#include "movement.h"

using namespace vex;

namespace {
double driveKP = DRIVE_KP, driveKI = DRIVE_KI, driveKD = DRIVE_KD;
double turnKP = TURN_KP, turnKI = TURN_KI, turnKD = TURN_KD;

double clampValue(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
double toRadians(double d) { return d * M_PI / 180.0; }
double toDegrees(double r) { return r * 180.0 / M_PI; }

void spinSigned(motor_group &group, double pct) {
    pct = clampValue(pct, -100.0, 100.0);
    if (pct >= 0) group.spin(forward, pct, percent);
    else group.spin(reverse, -pct, percent);
}
void spinSigned(motor &m, double pct) {
    pct = clampValue(pct, -100.0, 100.0);
    if (pct >= 0) m.spin(forward, pct, percent);
    else m.spin(reverse, -pct, percent);
}
bool waitForPosition(motor &m, double target, int timeoutMs) {
    int elapsed = 0;
    while (elapsed < timeoutMs) {
        if (std::abs(m.position(degrees) - target) <= 3.0) return true;
        wait(10, msec);
        elapsed += 10;
    }
    return false;
}
}

void setDrivePID(double p, double i, double d) {
    driveKP = p; driveKI = i; driveKD = d;
}
void setTurnPID(double p, double i, double d) {
    turnKP = p; turnKI = i; turnKD = d;
}

void resetOdometry(double x, double y, double headingDegrees) {
    odomUpdatesEnabled = false;
    wait(25, msec); // let any in-flight 20 ms update finish before resetting
    odom.reset(x, y, toRadians(headingDegrees));
    odomUpdatesEnabled = true;
}

void stopDrive(brakeType mode) {
    leftDrive.stop(mode);
    rightDrive.stop(mode);
}

bool driveDistance(double inches, double maxSpeed, int timeoutMs) {
    if (std::abs(inches) < 0.01) { stopDrive(brake); return true; }
    maxSpeed = clampValue(std::abs(maxSpeed), 8.0, 100.0);

    const double startX = odom.x, startY = odom.y, startHeading = odom.theta;
    double previousError = inches, integral = 0.0, commanded = 0.0;
    int elapsed = 0, settled = 0;
    const double dt = 0.02;

    while (elapsed < timeoutMs) {
        const double dx = odom.x - startX, dy = odom.y - startY;
        const double travelled = dx * std::sin(startHeading) + dy * std::cos(startHeading);
        const double error = inches - travelled;
        integral = clampValue(integral + error * dt, -20.0, 20.0);
        const double derivative = (error - previousError) / dt;

        // Speed profile limits output near the target; slew limiting ramps it.
        const double remainingCap = clampValue(std::abs(error) * 7.0, 9.0, maxSpeed);
        double output = driveKP * error + driveKI * integral + driveKD * derivative;
        output = clampValue(output, -remainingCap, remainingCap);
        if (std::abs(error) > 0.6 && std::abs(output) < 8.0)
            output = (output < 0 ? -8.0 : 8.0);
        const double maxStep = DRIVE_ACCEL_PERCENT_PER_SEC * dt;
        commanded += clampValue(output - commanded, -maxStep, maxStep);

        const double headingError = toDegrees(std::remainder(startHeading - odom.theta, 2.0 * M_PI));
        const double correction = clampValue(DRIVE_HEADING_KP * headingError, -12.0, 12.0);
        spinSigned(leftDrive, commanded + correction);
        spinSigned(rightDrive, commanded - correction);

        if (std::abs(error) <= 0.6) {
            if (++settled >= 8) { stopDrive(brake); return true; }
        } else settled = 0;

        previousError = error;
        wait(20, msec);
        elapsed += 20;
    }
    stopDrive(brake);
    return false;
}

bool turnToAngle(double targetDegrees, double maxSpeed, int timeoutMs) {
    maxSpeed = clampValue(std::abs(maxSpeed), 8.0, 100.0);
    const double target = toRadians(targetDegrees);
    double previousError = toDegrees(std::remainder(target - odom.theta, 2.0 * M_PI));
    double integral = 0.0;
    int elapsed = 0, settled = 0;
    const double dt = 0.02;

    while (elapsed < timeoutMs) {
        const double error = toDegrees(std::remainder(target - odom.theta, 2.0 * M_PI));
        integral = clampValue(integral + error * dt, -30.0, 30.0);
        const double derivative = (error - previousError) / dt;
        double output = turnKP * error + turnKI * integral + turnKD * derivative;
        const double cap = clampValue(std::abs(error) * 1.4, 8.0, maxSpeed);
        output = clampValue(output, -cap, cap);
        if (std::abs(error) > 1.5 && std::abs(output) < 7.0)
            output = (output < 0 ? -7.0 : 7.0);

        spinSigned(leftDrive, output);
        spinSigned(rightDrive, -output);
        if (std::abs(error) <= 1.5) {
            if (++settled >= 8) { stopDrive(brake); return true; }
        } else settled = 0;

        previousError = error;
        wait(20, msec);
        elapsed += 20;
    }
    stopDrive(brake);
    return false;
}

bool turnRelative(double degrees, double maxSpeed, int timeoutMs) {
    const double target = toDegrees(odom.theta) + degrees;
    return turnToAngle(target, maxSpeed, timeoutMs);
}

bool driveToPoint(double targetX, double targetY, double maxSpeed, int timeoutMs) {
    const double dx = targetX - odom.x, dy = targetY - odom.y;
    if (std::sqrt(dx * dx + dy * dy) <= DRIVE_POINT_TOLERANCE_IN) {
        stopDrive(brake); return true;
    }
    const double desiredHeading = toDegrees(std::atan2(dx, dy));
    if (!turnToAngle(desiredHeading, TURN_MAX_SPEED_PERCENT, 4000)) {
        stopDrive(brake); return false;
    }

    // Re-evaluate distance after the turn, then drive a bounded straight segment.
    const double remainingX = targetX - odom.x, remainingY = targetY - odom.y;
    const double distance = std::sqrt(remainingX * remainingX + remainingY * remainingY);
    const bool ok = driveDistance(distance, maxSpeed, timeoutMs);
    stopDrive(brake);
    return ok;
}

void moveCascade(double speedPercent) { spinSigned(liftMotors, speedPercent); }
void moveLift(double speedPercent) { moveCascade(speedPercent); }
void moveClawAngle(double speedPercent) { spinSigned(clawAngleMotor, speedPercent); }
void moveClawGrabber(double speedPercent) { spinSigned(clawGrabberMotor, speedPercent); }
void moveIntake(double speedPercent) { spinSigned(intakeMotor, speedPercent); }

void stopCascade(brakeType mode) { liftMotors.stop(mode); }
void stopLift(brakeType mode) { stopCascade(mode); }
void stopClawAngle(brakeType mode) { clawAngleMotor.stop(mode); }
void stopClawGrabber(brakeType mode) { clawGrabberMotor.stop(mode); }
void stopIntake(brakeType mode) { intakeMotor.stop(mode); }

bool moveCascadeToPosition(double target, double speedPercent, int timeoutMs) {
    const double speed = clampValue(std::abs(speedPercent), 1.0, 100.0);
    liftMotor1.spinToPosition(target, degrees, false);
    liftMotor2.spinToPosition(target, degrees, false);
    int elapsed = 0;
    while (elapsed < timeoutMs) {
        if (std::abs(liftMotor1.position(degrees) - target) <= 3.0 &&
            std::abs(liftMotor2.position(degrees) - target) <= 3.0) {
            stopCascade(hold); return true;
        }
        wait(10, msec); elapsed += 10;
    }
    stopCascade(hold);
    return false;
}

bool moveClawAngleToPosition(double target, double speedPercent, int timeoutMs) {
    const double speed = clampValue(std::abs(speedPercent), 1.0, 100.0);
    clawAngleMotor.spinToPosition(target, degrees,false);
    const bool reached = waitForPosition(clawAngleMotor, target, timeoutMs);
    stopClawAngle(hold);
    return reached;
}
