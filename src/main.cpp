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
#include "robot/monte.hpp"
#include "robot/skills.h"
#include <iostream>
#include <type_traits>
#include "helpers.hpp"
#include "robot/auton.hpp"

// ========== AUTON SELECTOR ==========

// Array of autonomous routine names (for display only)
const char* autons[] = {
    "Right Full",
    "Right 7 Long",
    "Right Quick",
    "Left Full",
    "Left 7 Long",
    "Left Quick",
    "AWP",
    "Skills",
    "Telemetry",
    "MCL Test"
};
int autonIndex = 0;
const int numAutons = 10; // Number of auton routines


void initialize() {
    pros::lcd::initialize();

    chassis.calibrate();
    // Left button - cycle backward
    pros::lcd::register_btn0_cb([]() {
        autonIndex--;
        if (autonIndex < 0) {
            autonIndex = numAutons - 1; // Wrap around
        }
        pros::lcd::print(2, "Selected: %s", autons[autonIndex]);
    });
    
    // Center button - could be used to confirm or just display
    pros::lcd::register_btn1_cb([]() {
        pros::lcd::print(2, "Selected: %s", autons[autonIndex]);
    });
    
    // Right button - cycle forward
    pros::lcd::register_btn2_cb([]() {
        autonIndex++;
        if (autonIndex >= numAutons) {
            autonIndex = 0; // Wrap around
        }
        pros::lcd::print(2, "Selected: %s", autons[autonIndex]);
    });
    
    // Display initial selection
    pros::lcd::print(2, "Selected: %s", autons[autonIndex]);

    pros::Task mclTask(MCL::taskFn);
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
const int pimpJiggle = 15; 
void autonomous() {

switch(autonIndex) {
        case 0: // Right Full
          rightFull();
            break;

        case 1: // Right 7 Long
          right7();
            break;
        case 2: // Right Quick
          rightQuick();
            break;

        case 3: // Left Full
          leftFull();
            break;

        case 4: // Left 7 Long
          left7();
            break;

        case 5: // Left Quick
          leftQuick();
            break;
        case 6: // AWP
          soloAWP();
            break;
        case 7: // Skills
          skills1();
            break;
        case 8: // Telemetry
          telemetry();
            break;
        case 9: // MCL Test
          testMCL();
            break;
    }

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

//  pros::Task stallTask(stallRecoveryTask);



//OLD AUTO CODE DONT USE
    // //Start touching red park
    // chassis.setPose(-48,-12,180);
    // chassis.moveToPoint(-48, -46, 3000);
    // chassis.turnToHeading(270,2000);
    // //go to loader
    // loaderFork.extend();
    // intake();
    // pros::delay(500);
    // //wiggle in - - match loader
    // chassis.moveToPoint(-75,-46.5,3000);
    // pros::delay(500);
    // chassis.moveToPoint(75,-46.5,150, {.forwards = false});
    // pros::delay(200);
    // chassis.moveToPoint(-75,-46.5,3000);
    // chassis.moveToPoint(75,-46.5,150, {.forwards = false});
    // pros::delay(200);
    // chassis.moveToPoint(-75,-46.5,3000);
    // pros::delay(4750);
    // //go to + - to score
    // chassis.moveToPoint(-48,-50,3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    // chassis.moveToPoint(-27.5, -62, 3000, {.forwards = false});
    // chassis.moveToPoint(45, -62, 4000, {.forwards = false});
    // loaderFork.retract();
    // chassis.turnToHeading(0,500);
    // chassis.moveToPoint(48,-48,2000);
    // chassis.turnToHeading(90,500);
    // // chassis.waitUntilDone();
    // // distanceCode("East");
    // // pros::delay(200);
    //         //     // print robot location to the brain screen
    //         // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // chassis.moveToPoint(0,-48,8000,{.forwards = false}, true);
    // pros::delay(1000);
    // topOuttake();
    // loaderFork.extend();
    // pros::delay(5000);
    // //grab + - match loader and wiggle
    // intake();
    // chassis.moveToPose(80,-47.5,90,1500,{}, true);
    // pros::delay(1500);
    // chassis.moveToPose(-80,-47.5,90,150, {.forwards = false});
    // chassis.moveToPose(80,-47.5,90,5000);
    // chassis.moveToPose(-80,-47.5,90,150, {.forwards = false});
    // chassis.moveToPose(80,-47.5,90,5000);
    // //score in + -
    // chassis.moveToPoint(0,-49,8000,{.forwards = false});
    // pros::delay(1000);
    // topOuttake();
    // pros::delay(5000);
    // //go to + + match loader
    // intake();
    // chassis.moveToPoint(40,-45,2000);
    // chassis.turnToHeading(0,1000);
    // chassis.moveToPoint(40,48,2500, {.maxSpeed = 100});
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(70, 48, 3000, {}, false);
    // //match load + + and wiggle
    // pros::delay(1000);
    // chassis.moveToPoint(-70,48,100,{.forwards = false});
    // chassis.moveToPoint(70, 48, 5000, {}, false);
    // loaderFork.retract();
    // //go to - + and score
    // chassis.moveToPoint(40,48,2500, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    // chassis.moveToPoint(25,61,2000);
    // chassis.moveToPoint(-48,61,5000);
    // chassis.moveToPoint(-48,48,2000);
    // chassis.moveToPoint(70,48,2000, {.forwards = false}, true);
    // pros::delay(950);
    // topOuttake();
    // //grab - + match loader
    // loaderFork.extend();
    // pros::delay(7000);
    // intake();
    // //wiggle
    // chassis.moveToPoint(-75,48,2000);
    // chassis.moveToPoint(75,48,150, {.forwards = false});
    // chassis.moveToPoint(-75,48,2000);
    // chassis.moveToPoint(75,48,150, {.forwards = false});
    // chassis.moveToPoint(-75,48,7000);
    // //score - +
    // chassis.moveToPoint(75,48,7000, {.forwards = false},true);
    // pros::delay(850);
    // topOuttake();
    // pros::delay(7000);
    // //park and clear parking
    // chassis.moveToPoint(-63,21,3000);
    // chassis.moveToPoint(-64,-100,500,{.minSpeed = 127});  




// //PRINT VALUES
      // chassis.setPose(0,0,0);
      // while(1){
      //                     // print robot location to the brain screen
      //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
      //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
      //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
              pros::lcd::print(3, "westPredict: %f", dWest.get_distance()/25.4+wDistCenter-70); // w
      //         pros::lcd::print(4, "eastPredict: %f", dEast.get_distance()/25.4+eDistCenter-70); // e
      //         pros::lcd::print(5, "sotuhPredict: %f", 70-dSouth.get_distance()/25.4-sDistCenter); // s  
      //         pros::lcd::print(9, "nortActual: %f", dNorth.get_distance()/25.4); // n
      //         pros::lcd::print(3, "eastActual: %f", dEast.get_distance()/25.4); // e
      //         pros::lcd::print(6, "southActual: %f", dSouth.get_distance()/25.4); // s
      //         pros::delay(100);
      // }




//TESTING SQUARE
    // for(int x = 0; x<4;x++){
    // chassis.moveToPoint(0,24,10000);
    // chassis.turnToHeading(90,5000);
    // chassis.moveToPoint(24,24,100000);
    // chassis.turnToHeading(180,5000);
    // chassis.moveToPoint(24,0,10000);
    // chassis.turnToHeading(270,5000);
    // chassis.moveToPoint(0,0,10000);
    // chassis.turnToHeading(0,5000);
    // }



//AUTON TESTING







// //DRIVE CODE
//     right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//     left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//     // loop forever
//     Descore.extend();
//     loaderFork.extend();
//     while (true) {
//         // get left y and right x positions
//         int leftY = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))*.8;
//         int rightX = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))*.8;
//         // move the robot
//         chassis.arcade(leftY, rightX);
//         controllerCode();
//         pros::delay(20);// small delay for loop timing
//     }
}