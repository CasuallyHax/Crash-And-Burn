#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#pragma once


// //Intake motors
extern pros::Motor intakeMotor;
extern pros::Motor hoodMotor;

// //pneumatics
// extern pros::adi::Pneumatics Hood;
extern pros::adi::Pneumatics loaderFork;
extern pros::adi::Pneumatics Descore;
extern pros::adi::Pneumatics midScore;

// //optical
// extern pros::Optical colorSorter;

//controller
extern pros::Controller controller;
extern pros::MotorGroup left_motors; // left motors on ports 1, 2, 3
extern pros::MotorGroup right_motors; // right motors on ports 4, 5, 6

extern lemlib::Drivetrain drivetrain;
extern pros::Imu imu;
extern pros::Rotation vertical_tracker;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;


extern pros::Distance dNorth;
extern pros::Distance dEast;
extern pros::Distance dSouth;
extern pros::Distance dWest;