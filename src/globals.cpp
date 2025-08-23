#include "globals.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"


//Intake motors
pros::Motor Intake_Bottom_11W(-2,pros::v5::MotorGears::blue);
pros::Motor Intake_Middle_55W(3, pros::v5::MotorGears::green);
pros::Motor Intake_Top_11W(5,pros::v5::MotorGears::blue);
pros::Motor Bucket_55W(6,pros::v5::MotorGearset::green);

//Limiting voltage of 5.5 watt motors
void InitializeMotors(){
    Intake_Middle_55W.set_voltage_limit(7200);
    Bucket_55W.set_voltage_limit(7200);
}

//pneumatics
pros::adi::Pneumatics LoaderFork('A', false);
pros::adi::Pneumatics Aligner('B', true);

//optical
pros::Optical ColorSorter(15);

// define the global variable
Color TeamColor = Color::RED; // default team

