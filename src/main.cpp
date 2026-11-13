
#include "override.h"
#include "movement.h"

#include <cmath>

using namespace vex;

namespace {

enum ScoreState {
    SCORE_IDLE,
    SCORE_OUTTAKE,
    SCORE_ANGLE,
    SCORE_CASCADE
};

ScoreState scoreState = SCORE_IDLE;

int scoreElapsedMs = 0;
bool xWasPressed = false;
bool arcadeDrive = true;

const double driveSpeed = 0.5;

RisingEdge btnB;

// Stop grabber and intake after outtaking.
void stopOuttake() {
    stopClawGrabber(hold);
    stopIntake(brake);
}

// Safely cancel the scoring sequence.
void abortScoreSequence() {
    stopOuttake();
    stopClawAngle(hold);
    stopCascade(hold);

    scoreState = SCORE_IDLE;
    scoreElapsedMs = 0;
}

// Nonblocking automatic scoring sequence.
void updateScoreSequence(bool xPressed) {

    const bool xRising = xPressed && !xWasPressed;
    xWasPressed = xPressed;

    // Start only on a new X press.
    if (scoreState == SCORE_IDLE && xRising) {
        scoreState = SCORE_OUTTAKE;
        scoreElapsedMs = 0;
    }

    switch (scoreState) {

    case SCORE_IDLE:
        break;

    // STEP 1: Reverse grabber and intake.
    case SCORE_OUTTAKE:

        moveClawGrabber(-70);
        moveIntake(-70);

        scoreElapsedMs += 20;

        if (scoreElapsedMs >= 500) {

            stopOuttake();

            // STEP 2: Move claw angle to maximum position.
            clawAngleMotor.spinToPosition(
                CLAW_MAX_ANGLE_DEG,
                degrees,
                30,
                velocityUnits::pct,
                false
            );

            scoreState = SCORE_ANGLE;
            scoreElapsedMs = 0;
        }

        break;

    // STEP 3: Wait for claw angle to reach target.
    case SCORE_ANGLE:

        scoreElapsedMs += 20;

        if (
            std::abs(
                clawAngleMotor.position(degrees)
                - CLAW_MAX_ANGLE_DEG
            ) <= 3.0
        ) {

            stopClawAngle(hold);

            // STEP 4: Move BOTH lift motors to cascade zero.
            // Cascade and lift are the SAME mechanism.
            liftMotors.spinToPosition(
                CASCADE_ZERO_DEG,
                degrees,
                30,
                velocityUnits::pct,
                false
            );

            scoreState = SCORE_CASCADE;
            scoreElapsedMs = 0;
        }
        else if (scoreElapsedMs >= 4000) {
            abortScoreSequence();
        }

        break;

    // STEP 5: Wait for both lift motors to reach zero.
    case SCORE_CASCADE:

        scoreElapsedMs += 20;

        if (
            std::abs(
                liftMotor1.position(degrees)
                - CASCADE_ZERO_DEG
            ) <= 3.0
            &&
            std::abs(
                liftMotor2.position(degrees)
                - CASCADE_ZERO_DEG
            ) <= 3.0
        ) {

            stopCascade(hold);

            scoreState = SCORE_IDLE;
            scoreElapsedMs = 0;
        }
        else if (scoreElapsedMs >= 5000) {
            abortScoreSequence();
        }

        break;
    }
}

// Calibrate the inertial sensor.
void calibrateImu() {

    IMU.calibrate();

    while (IMU.isCalibrating()) {
        wait(10, msec);
    }
}

}

// PRE-AUTONOMOUS
void pre_auton(void) {

    calibrateImu();
    resetOdometry();
}

// AUTONOMOUS
void autonomous(void) {

    resetOdometry();

    // Insert your competition autonomous route here.
    //
    // Example:
    // if (!driveDistance(24)) return;
    // if (!turnToAngle(90)) return;
    // if (!driveDistance(12)) return;

    stopDrive(brake);
    stopCascade(hold);
    stopClawAngle(hold);
    stopClawGrabber(hold);
    stopIntake(brake);
}

// DRIVER CONTROL
void usercontrol(void) {

    resetOdometry();

    scoreState = SCORE_IDLE;
    xWasPressed = false;

    while (true) {

        // ---------------------------------
        // ODOMETRY SCREEN
        // ---------------------------------

        static int displayTimer = 0;

        if (++displayTimer >= 5) {

            displayTimer = 0;

            Controller1.Screen.clearScreen();

            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print(
                "X:%.1f Y:%.1f",
                odom.x,
                odom.y
            );

            Controller1.Screen.setCursor(2, 1);
            Controller1.Screen.print(
                "H:%.1f deg",
                odom.theta * 180.0 / M_PI
            );
        }

        // ---------------------------------
        // DRIVE CONTROL
        // ---------------------------------

        if (btnB.check(Controller1.ButtonB.pressing())) {
            arcadeDrive = !arcadeDrive;
        }

        if (arcadeDrive) {

            const double fwd =
                Controller1.Axis3.position(percent);

            const double turn =
                0.5 * Controller1.Axis1.position(percent);

            leftDrive.spin(
                forward,
                driveSpeed * (fwd + turn),
                percent
            );

            rightDrive.spin(
                forward,
                driveSpeed * (fwd - turn),
                percent
            );

        } else {

            const double left =
                Controller1.Axis3.position(percent);

            const double right =
                Controller1.Axis2.position(percent);

            leftDrive.spin(
                forward,
                driveSpeed * left,
                percent
            );

            rightDrive.spin(
                forward,
                driveSpeed * right,
                percent
            );
        }

        // ---------------------------------
        // AUTOMATIC X SCORING
        // ---------------------------------

        updateScoreSequence(
            Controller1.ButtonX.pressing()
        );

        // ---------------------------------
        // MECHANISM CONTROLS
        // ---------------------------------

        // Manual mechanism controls are disabled
        // while the automatic scoring sequence runs.

        if (scoreState == SCORE_IDLE) {

            // CASCADE / LIFT
            // R1 = UP
            // L1 = DOWN

            if (Controller1.ButtonR1.pressing()) {

                moveCascade(75);

            }
            else if (Controller1.ButtonL1.pressing()) {

                moveCascade(-75);

            }
            else {

                stopCascade(hold);
            }

            // CLAW ANGLE
            // R2 = OUTWARD
            // L2 = INWARD

            if (Controller1.ButtonR2.pressing()) {

                moveClawAngle(45);

            }
            else if (Controller1.ButtonL2.pressing()) {

                moveClawAngle(-45);

            }
            else {

                stopClawAngle(hold);
            }

            // GRABBER + INTAKE
            // LEFT ARROW = FORWARD
            // A = REVERSE

            if (Controller1.ButtonLeft.pressing()) {

                moveClawGrabber(70);
                moveIntake(70);

            }
            else if (Controller1.ButtonA.pressing()) {

                moveClawGrabber(-70);
                moveIntake(-70);

            }
            else {

                stopClawGrabber(hold);
                stopIntake(brake);
            }
        }

        wait(20, msec);
    }
}

// MAIN
int main() {

    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true) {
        wait(100, msec);
    }
}
