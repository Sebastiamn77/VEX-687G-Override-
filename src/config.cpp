#include "override.h"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller(primary);

// -----------------------------------------------------------------------------
// HARDWARE CONFIGURATION — REVIEW BEFORE POWERING THE ROBOT
// liftMotor2 form the single liftMotors group. There is no separate cascade motor.
// -----------------------------------------------------------------------------
motor frontLeft  = motor(PORT5,  ratio6_1, true);
motor frontRight = motor(PORT6, ratio6_1, false);
motor backLeft   = motor(PORT3,  ratio6_1, true);
motor backRight  = motor(PORT4,  ratio6_1, false);

motor liftMotor1 = motor(PORT2, ratio6_1, false); // right moter of the lift, facing forwards (forward is intake) 
motor liftMotor2 = motor(PORT1, ratio6_1, true); 
motor_group liftMotors(liftMotor1, liftMotor2);
motor_group leftDrive(frontLeft, backLeft);
motor_group rightDrive(frontRight, backRight);

static const int CLAW_ANGLE_PORT_PLACEHOLDER = PORT11;
static const int CLAW_GRABBER_PORT_PLACEHOLDER = PORT12;
static const int INTAKE_PORT_PLACEHOLDER = PORT13;
motor clawAngleMotor(CLAW_ANGLE_PORT_PLACEHOLDER, ratio18_1, false);
motor clawGrabberMotor = (CLAW_GRABBER_PORT_PLACEHOLDER, ratio18_1, false);
motor intakeMotor = (PORT7, ratio18_1, false);

rotation forwardTracker = rotation(PORT5, false);
rotation lateralTracker = rotation(PORT6, false);
inertial IMU = inertial(PORT7);

// Tracking wheel diameter confirmed by project owner: 2.00 inches.
// Offsets are editable starting values, not a claim of final calibration.
const double ODOM_FORWARD_OFFSET_IN = 2.25;
const double ODOM_LATERAL_OFFSET_IN = 3.85;
Odometry odom(forwardTracker, lateralTracker, IMU, TRACKING_WHEEL_DIAMETER,
              ODOM_FORWARD_OFFSET_IN, ODOM_LATERAL_OFFSET_IN);

// Encoder-degree placeholders; calibrate mechanically before using X scoring.
const double CLAW_MAX_ANGLE_DEG = 0.0; // UNCALIBRATED motor encoder degrees
const double CASCADE_ZERO_DEG = 0.0;   // UNCALIBRATED motor encoder degrees

// Untested conservative starting gains. Tune on blocks in a safe area.
const double DRIVE_KP = 2.0;
const double DRIVE_KI = 0.0;
const double DRIVE_KD = 0.08;
const double TURN_KP = 0.85;
const double TURN_KI = 0.0;
const double TURN_KD = 0.06;
const double DRIVE_HEADING_KP = 0.8;
const double DRIVE_POINT_TOLERANCE_IN = 1.0;
const double DRIVE_ACCEL_PERCENT_PER_SEC = 90.0;
const double DRIVE_MAX_SPEED_PERCENT = 65.0;
const double TURN_MAX_SPEED_PERCENT = 45.0;

// Exactly one background task updates odometry during autonomous and driver.
bool odomUpdatesEnabled = false;
thread odomThread(odomTask);

int odomTask() {
    while (true) {
        if (odomUpdatesEnabled) odom.update();
        this_thread::sleep_for(20);
    }
    return 0;
}
