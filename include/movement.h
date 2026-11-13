#pragma once

#include "config.h"

// Distances are inches; angles are degrees. Blocking autonomous commands
// return false on timeout. Call them from autonomous, not the driver loop.
void setDrivePID(double kP, double kI, double kD);
void setTurnPID(double kP, double kI, double kD);
void resetOdometry(double x = 0.0, double y = 0.0, double headingDegrees = 0.0);

bool driveDistance(double inches, double maxSpeed = DRIVE_MAX_SPEED_PERCENT,
                   int timeoutMs = 5000);
bool turnToAngle(double degrees, double maxSpeed = TURN_MAX_SPEED_PERCENT,
                 int timeoutMs = 4000);
bool turnRelative(double degrees, double maxSpeed = TURN_MAX_SPEED_PERCENT,
                  int timeoutMs = 4000);
bool driveToPoint(double x, double y, double maxSpeed = DRIVE_MAX_SPEED_PERCENT,
                  int timeoutMs = 8000);

void stopDrive(vex::brakeType mode = vex::brake);
// Cascade and lift are the same mechanism; both names command liftMotors.
void moveCascade(double speedPercent);
void moveLift(double speedPercent); // compatibility alias for moveCascade()
void moveClawAngle(double speedPercent);
void moveClawGrabber(double speedPercent);
void moveIntake(double speedPercent);
void stopLift(vex::brakeType mode = vex::hold);
void stopCascade(vex::brakeType mode = vex::hold);
void stopClawAngle(vex::brakeType mode = vex::hold);
void stopClawGrabber(vex::brakeType mode = vex::hold);
void stopIntake(vex::brakeType mode = vex::brake);

bool moveCascadeToPosition(double encoderDegrees, double speedPercent = 35,
                           int timeoutMs = 4000);
bool moveClawAngleToPosition(double encoderDegrees, double speedPercent = 30,
                             int timeoutMs = 4000);
