#include "main.h"
#include "globals.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include "main.h"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "robot/auton.h"
//#include "robot/monte.hpp"
#include "robot/skills.h"
#include <iostream>
#include "helpers.hpp"


void initialize() {
    pros::lcd::initialize();

    chassis.calibrate();
/*    pros::Task([&] {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(10);
        }
    });*/
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
    chassis.setPose(-48,0,180);
    chassis.moveToPose(-48, -46.5,270, 10000);
    chassis.turnToHeading(270,2000);
    loaderFork.extend();
    intake();
    pros::delay(500);
    chassis.moveToPoint(-75,-46.5,3000);
    pros::delay(5000);
    chassis.moveToPoint(-48,-48,3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    chassis.moveToPoint(-27.5, -60, 3000, {.forwards = false});
    chassis.moveToPoint(45, -60, 4000, {.forwards = false});
    chassis.turnToHeading(0,500);
    chassis.moveToPoint(48,-48,2000);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(0,-48,8000,{.forwards = false}, true);
    topOuttake();
    pros::delay(5000);
    intake();
    chassis.moveToPose(80,-46.5,90,5000,{},false);
    chassis.moveToPoint(0,-48,8000,{.forwards = false}, true);
    topOuttake();
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
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // loop forever
    Descore.extend();
    loaderFork.extend();
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