#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(19);
inline pros::Motor topintake(20);
inline pros::Motor backintake(18);

inline ez::Piston matchload('H');

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');