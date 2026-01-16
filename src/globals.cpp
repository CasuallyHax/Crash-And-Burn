#include "globals.hpp"
#include "pros/abstract_motor.hpp"
 #include "pros/motors.hpp"


//Intake motors
pros::Motor intakeMotor(3, pros::v5::MotorGears::blue);
pros::Motor hoodMotor(19, pros::v5::MotorGears::blue);

// //pneumatics
// pros::adi::Pneumatics Hood('A', false);
pros::adi::Pneumatics loaderFork('B', false);
pros::adi::Pneumatics Descore('A', false);

pros::Controller controller(pros::E_CONTROLLER_MASTER);