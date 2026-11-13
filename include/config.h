#pragma once

#include "vex.h"

// -----------------------------------------------------------------------------
// CENTRAL HARDWARE / CALIBRATION CONFIGURATION
// Keep all port, reversal, gear-cartridge, and calibration edits in config.cpp.
// PLACEHOLDER ports and motor settings are NOT verified hardware assignments.
// Do not connect/power the robot until the conflict and placeholders are resolved.
// -----------------------------------------------------------------------------

extern vex::competition Competition;
extern vex::brain Brain;
extern vex::controller Controller1;

// Existing drivetrain (preserved from original project)
extern vex::motor frontLeft;
extern vex::motor frontRight;
extern vex::motor backLeft;
extern vex::motor backRight;
extern vex::motor_group leftDrive;
extern vex::motor_group rightDrive;

// Existing lift pair, used as the cascade assembly
extern vex::motor liftMotor1;
extern vex::motor liftMotor2;
extern vex::motor_group liftMotors;

// New mechanism motors (ports are explicitly UNVERIFIED placeholders)
extern vex::motor clawAngleMotor;
extern vex::motor clawGrabberMotor;
extern vex::motor intakeMotor;

// Tracking sensors
extern vex::rotation forwardTracker;
extern vex::rotation lateralTracker;
extern vex::inertial IMU;

class Odometry;
extern Odometry odom;
extern vex::thread odomThread;
extern bool odomUpdatesEnabled;
int odomTask();

// Editable starting calibration values (inches / motor encoder degrees).
extern const double ODOM_FORWARD_OFFSET_IN;
extern const double ODOM_LATERAL_OFFSET_IN;
extern const double CLAW_MAX_ANGLE_DEG;
extern const double CASCADE_ZERO_DEG;

// Conservative, UNTESTED PID starting values.
extern const double DRIVE_KP;
extern const double DRIVE_KI;
extern const double DRIVE_KD;
extern const double TURN_KP;
extern const double TURN_KI;
extern const double TURN_KD;
extern const double DRIVE_HEADING_KP;
extern const double DRIVE_POINT_TOLERANCE_IN;
extern const double DRIVE_ACCEL_PERCENT_PER_SEC;
extern const double DRIVE_MAX_SPEED_PERCENT;
extern const double TURN_MAX_SPEED_PERCENT;
