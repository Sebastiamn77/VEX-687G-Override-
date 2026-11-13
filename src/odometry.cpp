#include "override.h"

using namespace vex;

constexpr double DEG_TO_RAD = M_PI / 180.0; // constexpr evaluates the value at compile time

Odometry::Odometry(rotation &forwardWheel, rotation &lateralWheel, inertial &imu,
    double wheelDiameterInches, double fwdOffset, double latOffset)
    : x(0), y(0), theta(0),
    _fwd(forwardWheel), _lat(lateralWheel), _imu(imu),
    _wheelCircumference(M_PI * wheelDiameterInches),
    _fwdOffset(fwdOffset), _latOffset(latOffset),
    _prevFwd(0), _prevLat(0), _prevTheta(0), _thetaOffset(0) {}


// Default reset() sets the position to (0, 0, 0)
void Odometry::reset(double startX, double startY, double startTheta) {
    x = startX;
    y = startY;
    theta = startTheta;
    _prevFwd     = _fwd.position(degrees);
    _prevLat     = _lat.position(degrees);
    _thetaOffset = _imu.rotation(degrees) * DEG_TO_RAD - startTheta; // offset doesn't change until next reset, basically a new zero heading is set
    _prevTheta   = startTheta;
}

void Odometry::update() {
    double curFwd   = _fwd.position(degrees);
    double curLat   = _lat.position(degrees);
    double curTheta = _imu.rotation(degrees) * DEG_TO_RAD - _thetaOffset;

    double dTheta   = curTheta - _prevTheta;

    double deltaFwd = (curFwd - _prevFwd) / 360.0 * _wheelCircumference + _fwdOffset * dTheta;
    double deltaLat = (curLat - _prevLat) / 360.0 * _wheelCircumference - _latOffset * dTheta;

    double startTheta = _prevTheta;

    _prevFwd   = curFwd;
    _prevLat   = curLat;
    _prevTheta = curTheta;
    theta      = curTheta;

    double localX, localY;
    if (std::abs(dTheta) < 1e-9) { // safeguard to prevent division by zero
        localX = deltaLat;
        localY = deltaFwd;
    } else {
        double sinScale = sin(dTheta) / dTheta;
        double cosScale = (1.0 - cos(dTheta)) / dTheta;
        localX =  deltaLat * sinScale - deltaFwd * cosScale;
        localY =  deltaFwd * sinScale + deltaLat * cosScale;
    }

    double sinH = sin(startTheta);
    double cosH = cos(startTheta);

    x += localY * sinH + localX * cosH;
    y += localY * cosH - localX * sinH;
}