#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(7);
inline pros::Motor topintake(6);
inline pros::Motor backintake(18);

inline ez::Piston matchload('A');
inline ez::Piston descore('B');

inline ez::Piston med('D');
inline ez::Piston small('C');

pros::Distance rightDS(10);

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');