#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp" // IWYU pragma: keep
#include "helpers.hpp"
#include "globals.hpp" // IWYU pragma: keep
#include "colorSorting.hpp"
//ASSET(PushBackAutons1);

pros::MotorGroup left_motors({-13,-14}, pros::MotorGearset::green); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({19,17}, pros::MotorGearset::green); // right motors on ports 4, 5, 6

//creating intake motor group

// create a v5 rotation sensor on port 1
pros::Rotation vertical_tracker(20);

// create a v5 rotation sensor on port 1
pros::Rotation horizontal_tracker(10);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              200, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// create an imu on port 10
pros::Imu imu(3);

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
// void initialize() {
// 	pros::lcd::initialize();
// 	pros::lcd::set_text(1, "Hello PROS User!");

// 	pros::lcd::register_btn1_cb(on_center_button);
// }

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracker, lemlib::Omniwheel::NEW_275, -1);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracker, lemlib::Omniwheel::NEW_275, -7.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
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

    colorSorter.set_led_pwm(0);

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
    //IMPORTANT
    //NEED TO CHANGE STARTING POS
    chassis.setPose(-58,-13,180);

    chassis.moveToPoint(-58,-20,2000);
    //PID tuning, comment out when not using
    //Lateral PID tuning
    //    chassis.moveToPoint(0,24, 10000);

    //Angular PID tuning
    //    chassis.follow(PushBackAutons1, 15, 20000);



    //Skills Option A
    // //go to closest loader
    // colorSorting();
    // Hood.extend();
    // chassis.moveToPose(0,0,0,4000);
    // //back out of it
    // chassis.moveToPose(0,0,0,4000, {.forwards = false}); 
    // chassis.moveToPose(0,0,90,4000);
    // //go to other two reds
    // chassis.moveToPose(0,0,0,4000);
    // //grab them
    // chassis.moveToPose(0,0,0,4000);
    // //score blocks in middle zone
    // Aligner.retract();
    // Hood.extend();
    // chassis.moveToPose(0,0,0,4000);
    // MiddleScoring();
    // //back up
    // chassis.moveToPose(0,0,0,4000, {.forwards = false});
    // Aligner.extend();
    // Hood.retract();
    // colorSorting();
    // //go to parking area to grab 6 blocks from there
    // chassis.moveToPose(0,0,0,4000);
    // //Move out of parking to release fork
    // chassis.moveToPose(0,0,0,4000);
    // Hood.extend();
    // chassis.moveToPose(0,0,0,4000);
    // //go to loader zone and only grab three blocks, mess with a delay to consistently only grab three
    // chassis.moveToPose(0,0,0,4000);
    // //back up
    // chassis.moveToPose(0,0,0,4000, {.forwards = false});
    // //score in long goal
    // Aligner.extend();
    // Hood.retract();
    // chassis.moveToPose(0,0,0,4000);
    // TopScoring();
    // //back up
    // chassis.moveToPose(0,0,0,4000, {.forwards = false});
    // Color TeamColor = Color::BLUE;
    // Hood.extend();
    // colorSorting();
    // //grab the other three blues from loading zone
    // chassis.moveToPose(0,0,0,4000);
    // Hood.retract();
    // //position robot to grab two blues
    // chassis.moveToPose(0,0,0,4000);
    // //grab other two blues
    // chassis.moveToPose(0,0,0,4000);
    // chassis.moveToPose(0,0,0,4000);
    // //score them in bottom goal
    // chassis.moveToPose(0,0,0,4000);
    // chassis.moveToPose(0,0,0,4000);
    // BottomScoring();
    // //back up
    // chassis.moveToPose(0,0,0,4000, {.forwards = false});
    // //park
    // chassis.moveToPose(0,0,0,4000);



    // //Skills Option B
    // //grab from bottom loader
    // Hood.extend();
    // colorSorting();
    // chassis.moveToPose(0,0,0,4000);
    // //back up
    // chassis.moveToPose(0,0,0,4000, {.forwards = false});
    // Hood.retract();
    // //grab two reds (bottom left)
    // chassis.moveToPose(0,0,0,4000);
    // //grab other two reds (bottom right)
    // chassis.moveToPose(0,0,0,4000);
    // chassis.moveToPose(0,0,0,4000);
    // //grab other two reds and score in bottom (top right)
    // chassis.moveToPose(0,0,0,4000);
    // chassis.moveToPose(0,0,0,4000);
    // BottomScoring();
    // //back up
    // chassis.moveToPose(0,0,0,4000, {.forwards = false});
    // //go to loader
    // Hood.extend();
    // colorSorting();
    // chassis.moveToPose(0,0,0,4000);
    // //back up
    // chassis.moveToPose(0,0,0,4000, {.forwards = false});
    // //grab other two reds (top left)
    // chassis.moveToPose(0,0,0,4000);
    // //score in middle (top left)
    // Aligner.retract();
    // Hood.extend();
    // chassis.moveToPose(0,0,0,4000);
    // MiddleScoring();
    // //unload loader (top left)
    // Hood.extend();
    // IntakeToBucket();
    // chassis.moveToPose(0,0,0,4000);
    // //back up
    // chassis.moveToPose(0, 0, 0, 4000, {.forwards = false});
    // Hood.retract();
    // //park
    // chassis.moveToPose(0,0,0,4000)

//Match Autons
//Score Preload in middle 
chassis.moveToPose(0,0,0,4000);
MiddleScoring();
chassis.moveToPose(0,0,0,4000);
pros::delay(400);
IntakeToBucket();
//back out
chassis.moveToPose(0,0,0,4000, {.forwards = false});
//pick up blocks
chassis.moveToPose(0,0,0,4000);
chassis.moveToPose(0,0,0,4000);
chassis.moveToPose(0,0,0,4000);
//score in bottom
chassis.moveToPose(0,0,0,4000);
BottomScoring();
pros::delay(600);
//back out
chassis.moveToPose(0,0,0,4000, {.forwards = false});
//go to loader
chassis.moveToPose(0,0,0,4000);
LoaderFork.extend();
chassis.moveToPose(0,0,0,4000);
//back out
chassis.moveToPose(0,0,0,4000, {false});
//score in top goal
chassis.moveToPose(0,0,0,4000);
TopScoring();
chassis.moveToPose(0,0,0,4000);
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
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        controllerCode();

        pros::delay(20);// small delay for loop timing
    }
}