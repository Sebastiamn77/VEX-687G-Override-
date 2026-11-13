#pragma once

#include "override.h"

// RISING EDGE
// Rising edge is so annoying to implement constantly in main.cpp so I just made this in case I used it more in the future
struct RisingEdge {
    bool prev = false;
    bool check(bool current) {
        bool triggered = current && !prev;
        prev = current;
        return triggered;
    }
};

// Tanh clamp function used for soft clamping such as in the PID Controller
inline double clamp(double value, double limit) {
        return std::tanh(value / limit) * limit;
}

// Wraps an angle to the range [-pi, pi)
inline double wrapAngle(double angle) {
    return std::remainder(angle, 2 * M_PI);
}