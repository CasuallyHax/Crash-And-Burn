#include "globals.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"

//Intake motors
pros::Motor Intake_Bottom_11W(-11,pros::v5::MotorGears::blue);
pros::Motor Intake_Middle_55W(-16, pros::v5::MotorGears::green);
pros::Motor Intake_Top_11W(-10,pros::v5::MotorGears::blue);
pros::Motor Bucket_55W(20,pros::v5::MotorGearset::green);
pros::Motor Intake_Top_Front_55W(12, pros::v5::MotorGears::green);
pros::Motor Agitator_55W(9,pros::v5::MotorGears::green);
pros::Motor Intake_Top_Back_55W(7, pros::v5::MotorGears::green);

//Limiting voltage of 5.5 watt motors
void InitializeMotors(){
    Intake_Middle_55W.set_voltage_limit(7200);
    Bucket_55W.set_voltage_limit(7200);
    Intake_Top_Front_55W.set_voltage_limit(7200);
    Agitator_55W.set_voltage_limit(7200);
    Intake_Top_Back_55W.set_voltage_limit(7200);
}

//pneumatics
pros::adi::Pneumatics Hood('A', false);
pros::adi::Pneumatics LoaderFork('B', false);


//optical
pros::Optical colorSorter(21);


pros::Controller controller(pros::E_CONTROLLER_MASTER);