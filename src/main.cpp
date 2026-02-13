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
//#include "robot/monte.hpp"
#include "robot/skills.h"
#include <iostream>
#include "helpers.hpp"
#include "robot/auton.hpp"


void on_left() {
  auton--;
  if (auton < 0) auton = AUTON_COUNT - 1;
  pros::lcd::set_text(1, autonNames[auton]);
}

void on_right() {
  auton++;
  if (auton >= AUTON_COUNT) auton = 0;
  pros::lcd::set_text(1, autonNames[auton]);
}

void on_center() {
  pros::lcd::set_text(2, "Selected!");
}


void initialize() {
    pros::lcd::initialize();

    chassis.calibrate();
  pros::lcd::set_text(0, "Auton Selector");
  pros::lcd::set_text(1, autonNames[auton]);

  pros::lcd::register_btn0_cb(on_left);    // left
  pros::lcd::register_btn1_cb(on_center);  // center
  pros::lcd::register_btn2_cb(on_right);   // right
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
  // chassis.setPose(0,0,0);
  // chassis.moveToPoint(0,24,1000);
  // chassis.setPose(-48,12,0);
  // //grab balls from loader
  // chassis.moveToPoint(-48, 48, 3000);
  // chassis.turnToHeading(270,2000);
  // loaderFork.extend();
  // intake();
  // chassis.moveToPoint(-70,47.5,4500,{},true);
  // //tune delay to only grab 3
  // pros::delay(4500);
  // //score 4 in long
  // chassis.moveToPoint(70,47.5,1000,{.forwards = false},false);
  // topOuttake();
  // pros::delay(3000);
  // loaderFork.retract();
  // intake();
  // chassis.moveToPoint(-48,48,400);
  // chassis.turnToHeading(135,250);
  // chassis.moveToPoint(-17,17,700,{},true);
  // //tune delay
  // pros::delay(800);
  // loaderFork.extend();
  // pros::delay(300);
  // //turn around to mid score 3
  // chassis.turnToHeading(315,400);
  // chassis.moveToPoint(-10,8.5,1000, {.forwards = false, .maxSpeed = 100}, false);
  // chassis.turnToHeading(315,400);  
  // midOuttake();
  // pros::delay(1500);
  // chassis.moveToPoint(-90,90,50, {}, false); 
  // chassis.moveToPoint(-9,9,1000, {.forwards = false}, false); 
  // //go to defend
  // loaderFork.retract();
  // chassis.moveToPose(-27.5,36,270, 750, {.minSpeed = 72});
  // Descore.retract();
  // left_motors.set_brake_mode_all(pros::MotorBrake::hold);
  // right_motors.set_brake_mode_all(pros::MotorBrake::hold);
  // chassis.moveToPoint(-11,36,400, {.forwards = false});


//   switch (auton) {
//     case 0:
//       rightFull();
//       break;
//     case 1:
//       leftFull();
//       break;
//     case 2:
//       rightQuick();
//       break;
//     case 3:
//       leftQuick();
//       break;
//     case 4:
//       soloAWP();
//       break;
//     case 5:
//       skills1();
//       break;
//     case 6:
//       drive24();
//       break;
//     case 7:
//       turn90();
//       break;
// }

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
const int pibJiggle = 30;
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
//       chassis.setPose(0,0,0);
//       while(1){
//                           // print robot location to the brain screen
//               pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
//               pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
//               pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
//               pros::lcd::print(3, "northPredict: %f", dNorth.get_distance()/25.4+nDistCenter-70); // n
//               pros::lcd::print(4, "eastPredict: %f", dEast.get_distance()/25.4+eDistCenter-70); // e
//               pros::lcd::print(7, "sotuhPredict: %f", dSouth.get_distance()/25.4+sDistCenter-70); // s  
//               pros::lcd::print(5, "nortActual: %f", dNorth.get_distance()/25.4); // n
//               pros::lcd::print(8, "eastActual: %f", dEast.get_distance()/25.4); // e
//               pros::lcd::print(6, "southActual: %f", dSouth.get_distance()/25.4); // s
//               pros::delay(100);
//       }




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
//TODO: move this code to auton after testing
    //Start touching red park
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Descore.extend();
    chassis.setPose(-48,-12,180);
    chassis.moveToPoint(-48, -46, 3000,{},false);
    chassis.waitUntilDone();
    pros::delay(200);
    distanceCode("--");
    pros::delay(2000);
    pros::lcd::print(0, "Done");
    chassis.turnToHeading(270,2000);
    //go to loader
    loaderFork.extend();
    intake();
    pros::delay(500);
    //wiggle in - - match loader
    pros::delay(1000);
    for(int move1=0;move1<pibJiggle;move1++){
      chassis.moveToPoint(-75,-46.5,150);
    pros::delay(200);
    }
    intakeStop();
    //go to + - to score
    chassis.moveToPoint(-48,-50,3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    loaderFork.retract();
    chassis.moveToPoint(-27.5, -60, 3000, {.forwards = false});
    chassis.moveToPoint(45, -60, 4000, {.forwards = false});
    chassis.moveToPoint(48,-48,500);
    chassis.moveToPoint(48,-100,2000,{.forwards = false, .maxSpeed = 75});
    chassis.swingToHeading(0,lemlib::DriveSide::LEFT,500);
    pros::delay(200);
    chassis.moveToPose(chassis.getPose().x,-48,0,1000);
    // chassis.waitUntilDone();
    // pros::delay(500);
    // distanceCode("+-");
    // pros::delay(2000);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(0,-48,3000,{.forwards = false}, true);
    pros::delay(1000);
    pros::delay(1000);
    topOuttake();
    chassis.turnToHeading(90,1000);
    chassis.setPose(chassis.getPose().x,dEast.get_distance()/25.4+eDistCenter-70,chassis.getPose().theta);
    pros::delay(2000);
    bottomOuttake();
    pros::delay(300);
    topOuttake();
    pros::delay(3000);
    loaderFork.extend();
    pros::delay(500);
    //grab + - match loader and wiggle
    intake();
    chassis.moveToPoint(80,posnegLoaderY,1700, {.maxSpeed= 40});  // match load +- quadrant
    for(int move1=0;move1<pibJiggle;move1++){
      chassis.moveToPoint(80,posnegLoaderY,150);
    pros::delay(200);
    }
    //score in + -
    chassis.moveToPoint(0,-49,3000,{.forwards = false});
    pros::delay(1000);
    topOuttake();
    chassis.turnToHeading(90,1000);
    chassis.setPose(chassis.getPose().x,dEast.get_distance()/25.4+eDistCenter-70,chassis.getPose().theta);
    pros::delay(2000);
    bottomOuttake();
    pros::delay(300);
    topOuttake();
    pros::delay(3000);
    //go to + + match loader
    intake();
    chassis.moveToPoint(35,-45,2000);
    chassis.moveToPoint(35,48,2000);
    chassis.turnToHeading(0,1000);
    chassis.waitUntilDone();
    pros::delay(200);
    distanceCode("++");
    pros::delay(2000);  // TODO: shorten after testing
    chassis.turnToHeading(90, 1000);
    //match load + + and wiggle
      chassis.moveToPoint(70,48,1700, {.maxSpeed= 50});
    for(int move1=0;move1<pibJiggle;move1++){
      chassis.moveToPoint(70,48,150);
      pros::delay(200);
    }
    //go to - + and score
    chassis.moveToPoint(40,48,2500, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    loaderFork.retract();
    chassis.moveToPoint(25,61,2000);
    chassis.moveToPoint(-48,61,5000);
    chassis.turnToHeading(180,500);
    chassis.moveToPoint(-48,48,750);
    chassis.turnToHeading(180,500);
    chassis.waitUntilDone();
    pros::delay(200);
    distanceCode("-+");
    pros::delay(2000);
    chassis.turnToHeading(270,500);
    chassis.moveToPoint(0,48,2000, {.forwards = false}, true);
    pros::delay(1000);
    topOuttake();
    chassis.turnToHeading(270,1000);
    chassis.setPose(chassis.getPose().x,70-dEast.get_distance()/25.4-eDistCenter,chassis.getPose().theta);
    pros::delay(2000);
    bottomOuttake();
    loaderFork.extend();
    pros::delay(300);
    topOuttake();
    pros::delay(3000);
    intake();
    pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    // //wiggle
    //   chassis.moveToPoint(-75,48,1700, {.maxSpeed= 50});
    // for(int move1=0;move1<pibJiggle;move1++){
    //   chassis.moveToPoint(-75,48,150);
    //   pros::delay(200);
    // }
    // pros::delay(5000);
    // //score - +
    // chassis.moveToPoint(75,50,3000, {.forwards = false},true);
    // pros::delay(1000);
    // topOuttake();
    // chassis.turnToHeading(270,1000);
    // chassis.setPose(chassis.getPose().x,70-dEast.get_distance()/25.4-eDistCenter,chassis.getPose().theta);
    // pros::delay(2000);
    // bottomOuttake();
    // pros::delay(300);
    // topOuttake();
    // pros::delay(3000);
    // loaderFork.retract();
    // //park and clear parking
    // chassis.moveToPoint(-63,21,3000);
    // chassis.moveToPoint(-64,-100,750,{.minSpeed = 127});  
  


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