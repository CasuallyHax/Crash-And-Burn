#include "globals.hpp"
#include "pros/motors.hpp"


//Intake motors
pros::Motor Intake_Bottom_11W(-2,pros::v5::MotorGears::blue);
pros::Motor* Intake_Middle_55W = nullptr;
pros::Motor Intake_Top_11W(5,pros::v5::MotorGears::blue);
pros::Motor* Bucket_55W = nullptr;

//Limiting voltage of 5.5 watt motors
void InitializeMotors(){
    Intake_Middle_55W = new pros::Motor(3, pros::v5::MotorGears::green);
    Intake_Middle_55W->set_voltage_limit(7200);
    Bucket_55W = new pros::Motor(6, pros::v5::MotorGears::green);
    Bucket_55W->set_voltage_limit(7200);
}

//pneumatics
pros::adi::Pneumatics LoaderFork('A', false);
pros::adi::Pneumatics Aligner('B', true);

//optical
pros::Optical ColorSorter(15);

// define the global variable
Color TeamColor = Color::RED; // default team

