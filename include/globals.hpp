#pragma once
#include "pros/motors.hpp"
#include "Pros/optical.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"

//Intake motors
extern pros::Motor Intake_Bottom_11W;
extern pros::Motor Intake_Middle_55W;
extern pros::Motor Intake_Top_11W;
extern pros::Motor Bucket_55W;
extern pros::Motor Intake_Top_Front_55W;
extern pros::Motor Agitator_55W;
extern pros::Motor Intake_Top_Back_55W;

void InitializeMotors();

//pneumatics
extern pros::adi::Pneumatics Hood;
extern pros::adi::Pneumatics LoaderFork;

//optical
extern pros::Optical colorSorter;

//controller
extern pros::Controller controller;