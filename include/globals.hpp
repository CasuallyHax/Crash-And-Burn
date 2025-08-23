#pragma once
#include "pros/motors.hpp"
#include "Pros/optical.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"

//Intake motors
extern pros::Motor Intake_Bottom_11W;
extern pros::Motor* Intake_Middle_55W;
extern pros::Motor Intake_Top_11W;
extern pros::Motor* Bucket_55W;

void InitializeMotors();

//pneumatics
extern pros::adi::Pneumatics LoaderFork;
extern pros::adi::Pneumatics Aligner;

//optical
extern pros::Optical ColorSorter;

// forward declare the enum
enum class Color { RED, BLUE, UNKNOWN };

// declare the global variable
extern Color TeamColor;