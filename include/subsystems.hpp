#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

pros::Motor intake(19,pros::MotorGearset::blue);
pros::Motor topintake(20,pros::MotorGearset::green);
pros::Motor backintake(18,pros::MotorGearset::green);

inline ez::Piston matchload('H');

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');