#include "globals.hpp"


//Intake motors
pros::Motor intakeMotor(3, pros::v5::MotorGears::blue);
pros::Motor hoodMotor(19, pros::v5::MotorGears::blue);

// //pneumatics
// pros::adi::Pneumatics Hood('A', false);
pros::adi::Pneumatics loaderFork('B', false);
pros::adi::Pneumatics Descore('A', false);
pros::adi::Pneumatics midScore('C', true);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-11,-1,-2}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({20,9,10}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// create an imu on port 10
pros::Imu imu(17);

pros::Rotation vertical_tracker(8);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracker, lemlib::Omniwheel::NEW_2*.925, 0);

// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracker, lemlib::Omniwheel::NEW_275, -7.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
                            
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
                        
);

pros::Distance dNorth(4);
pros::Distance dEast(14);
pros::Distance dSouth(12);
pros::Distance dWest(16);

//Auton selector
int auton = 0;

const char* autonNames[] = {
  "Skills",
  "Right Full",
  "Left Full",
  "Right Quick",
  "Left Quick",
  "Solo AWP",
  "Skills",
  "Drive 24in",
  "Turn 90deg"
};

const int AUTON_COUNT = 8;

