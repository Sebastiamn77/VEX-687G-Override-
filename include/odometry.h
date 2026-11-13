#pragma once

#include "override.h"

constexpr double TRACKING_WHEEL_DIAMETER = 2.00; // diameter of the tracking wheels in inches 2 inches based on Vex website

class Odometry {
public:
    double x;
    double y;
    double theta;

    Odometry(vex::rotation &forwardWheel, vex::rotation &lateralWheel,
        vex::inertial &imu, double wheelDiameterInches = TRACKING_WHEEL_DIAMETER,
        double fwdOffset = 0, double latOffset = 0);

    void reset(double startX = 0, double startY = 0, double startTheta = 0);
    void update();

private:
    vex::rotation &_fwd;
    vex::rotation &_lat;
    vex::inertial &_imu;

    double _wheelCircumference;
    double _fwdOffset; // PERPENDICULAR distance from forward tracking wheel to center of rotation, in inches
    double _latOffset; // same thing, but for lateral wheel

    // right and forward are positive offset

    double _prevFwd;
    double _prevLat;
    double _prevTheta;

    double _thetaOffset;
};
