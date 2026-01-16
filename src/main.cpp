#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp" // IWYU pragma: keep
#include "helpers.hpp"
#include "globals.hpp" // IWYU pragma: keep
#include "colorSorting.hpp"
#include "pros/rtos.h"
//ASSET(PushBackAutons1);

pros::MotorGroup left_motors({-11,-1,-2}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({20,9,10}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

// //creating intake motor group

// // create a v5 rotation sensor on port 1
// pros::Rotation vertical_tracker(20);

// // create a v5 rotation sensor on port 1
// pros::Rotation horizontal_tracker(10);

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

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracker, lemlib::Omniwheel::NEW_275, -1);

// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracker, lemlib::Omniwheel::NEW_275, -7.5);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
                            
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              70, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              70, // derivative gain (kD)
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


// this runs at the start of the program
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();

//    colorSorter.set_led_pwm(0);

    void InitializeMotors();

     pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources

    
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 
void autonomous() {
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //IMPORTANT
    chassis.setPose(0,0,0);

    //PID tuning, comment out when not using
    //Lateral PID tuning
        //move to loader
        chassis.moveToPoint(0,28.5, 10000, {.maxSpeed=90});
        chassis.turnToHeading(90,2000, {.maxSpeed=90});
        loaderFork.extend();
        pros::delay(200);
        intake();
        pros::delay(200);
        //grab balls and jiggle
        chassis.moveToPose(20,30.5,90,2000, {.maxSpeed=100});
        chassis.moveToPoint(-70,29, 250, {.forwards = false, .maxSpeed=100}, false);
        pros::delay(200);
        chassis.moveToPoint(20,30.5,2500, {.maxSpeed=100});
        chassis.moveToPoint(-70,29, 250, {.forwards = false, .maxSpeed=100},false);
        pros::delay(200);
        chassis.moveToPoint(20,30.5,3500, {.maxSpeed=100});
        //go to long goal
        chassis.moveToPoint(-70,32, 5000, {.forwards = false, .maxSpeed=90});
        pros::delay(1250);
        loaderFork.retract();
        topOuttake();
        pros::delay(5000);
        //park
        chassis.moveToPose(13,5,180,4500,{.maxSpeed=100}, false);
        chassis.moveToPoint(13,30,300,{.forwards = false});
        chassis.moveToPoint(13,-25,9000);

    //Angular PID tuning
   // chassis.turnToHeading(90,5000);

    //Skills Auton
    //take blocks from - - loader
    //     intake();
    //     loaderFork.extend();
    //     chassis.moveToPose(0,0,0,0);
    //     chassis.moveToPose(0,0,0,0);
    //     pros::delay(0);
    // //go to + - corner and score
    //     chassis.moveToPose(0,0,0,0);
    //     loaderFork.retract();
    //     chassis.moveToPose(0,0,0,0);
    //     topOuttake();
    //     pros::delay(0);
    // //take blocks from + - loader
    //     intake();
    //     loaderFork.extend();
    //     chassis.moveToPose(0,0,0,0);
    //     pros::delay(0);
    // //go to + - corner and score
    //     chassis.moveToPose(0,0,0,0, {false});
    //     loaderFork.retract();
    //     topOuttake();
    //     pros::delay(0);
    // //take blocks from + + loader
    //     intake();
    //     loaderFork.extend();
    //     chassis.moveToPose(0,0,0,0);
    //     chassis.moveToPose(0,0,0,0);
    //     pros::delay(0);
    // //go to - + corner and score
    //     chassis.moveToPose(0,0,0,0);
    //     loaderFork.retract();
    //     chassis.moveToPose(0,0,0,0);
    //     topOuttake();
    //     pros::delay(0);
    // //take blocks from - + loader
    //     intake();
    //     loaderFork.extend();
    //     chassis.moveToPose(0,0,0,0);
    //     pros::delay(0);
    // //go to - + corner and score
    //     chassis.moveToPose(0,0,0,0, {false});
    //     loaderFork.retract();
    //     topOuttake();
    //     pros::delay(0);
    // //go park and clear parking zone
    //     intake();
    //     chassis.moveToPose(0,0,0,0, {}, false);
    //     loaderFork.extend();
    //     chassis.turnToHeading(900000, 9000);


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))*.8;
        int rightX = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))*.8;

        // move the robot
        chassis.arcade(leftY, rightX);

        controllerCode();

        pros::delay(20);// small delay for loop timing
    }
}