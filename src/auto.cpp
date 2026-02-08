#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot/auton.hpp"
#include "helpers.hpp"
#include <cmath>
#include <iostream>
#include <string>

using namespace lemlib;

int distint = 0;
float nDistCenter = 4.25;
const float sDistCenter = 2.5;
const float eDistCenter = 3.25;
const float wDistCenter = 6.625;
double seventy = 70;
void distanceCode(std::string distance){

    int counter=1;
  if(distance == "--"){
    distint = 1;
  } else if(distance == "+-"){
    distint = 2;
  } else if (distance == "++"){
    distint = 3;
  } else if (distance == "-+"){
    distint = 4;}
  switch(distint){
    case 1:
    //--
    chassis.setPose(
    chassis.getPose().x,
    dNorth.get_distance()/25.4+nDistCenter-70,
    chassis.getPose().theta);
    if(chassis.getPose().y<-47){ // Closer to the wall, need to back up
      chassis.moveToPoint(-48,-46.5,500,{.forwards = false});
    }else if(chassis.getPose().y>-46){ // further from the wall, need to go forward
      chassis.moveToPoint(-48,-46.5,500);
    }
    break;
    
    case 2:
    //+-
    chassis.setPose(
    chassis.getPose().x,
    dSouth.get_distance()/25.4+sDistCenter-70,
    chassis.getPose().theta);
    if(chassis.getPose().y>-46){ // further from the wall, need to backup
      chassis.moveToPoint(48,-46.5,500,{.forwards = false});
    }else if(chassis.getPose().y<-47){ // closer to the wall, need to move forward
      chassis.moveToPoint(48,-46.5,500);
    }
    break;

    case 3:
    //++
    chassis.setPose(
    chassis.getPose().x,
    70-dNorth.get_distance()/25.4-nDistCenter,
    chassis.getPose().theta);
    if(chassis.getPose().y>48.5){ // closer to the wall, need to backup
      chassis.moveToPoint(48,48,500,{.forwards = false});
    }else if(chassis.getPose().y<47.5){ // further from the wall, need to move forward
      chassis.moveToPoint(48,48,500);
    }
    break;
    
    case 4:
    //-+
    chassis.setPose(
    chassis.getPose().x,
    70-dSouth.get_distance()/25.4-sDistCenter,
    chassis.getPose().theta);
    if(chassis.getPose().y<47.5){ // further from the wall, need to back up
      chassis.moveToPoint(-48,48,500,{.forwards = false});
    }else if(chassis.getPose().y>48.5){ // closer to the wall, need to move forward
      chassis.moveToPoint(-48,48,500);
    }
    break;
    }
  }

//match autons
void skills1(){
    //Start touching red park
    chassis.setPose(-48,-12,180);
    chassis.moveToPose(-48, -49,270, 3000);
    chassis.turnToHeading(270,2000);
    //go to loader
    loaderFork.extend();
    intake();
    pros::delay(500);
    //wiggle in - - match loader
    chassis.moveToPoint(-75,-46.5,3000);
    pros::delay(500);
    chassis.moveToPoint(75,-46.5,150, {.forwards = false});
    pros::delay(200);
    chassis.moveToPoint(-75,-46.5,3000);
    chassis.moveToPoint(75,-46.5,150, {.forwards = false});
    pros::delay(200);
    chassis.moveToPoint(-75,-46.5,3000);
    pros::delay(4750);
    //go to + - to score
    chassis.moveToPoint(-48,-50,3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    chassis.moveToPoint(-27.5, -62, 3000, {.forwards = false});
    chassis.moveToPoint(45, -62, 4000, {.forwards = false});
    chassis.waitUntilDone();
    distanceCode("East");
    pros::delay(200);
    chassis.turnToHeading(0,500);
    chassis.moveToPoint(48,-48,2000);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(0,-48,8000,{.forwards = false}, true);
    pros::delay(1000);
    topOuttake();
    pros::delay(5000);
    //grab + - match loader and wiggle
    intake();
    chassis.moveToPose(80,-46.5,90,1500,{}, true);
    pros::delay(1500);
    chassis.moveToPose(-80,-46.5,90,150, {.forwards = false});
    chassis.moveToPose(80,-46.5,90,5000);
    chassis.moveToPose(-80,-46.5,90,150, {.forwards = false});
    chassis.moveToPose(80,-46.5,90,5000);
    //score in + -
    chassis.moveToPoint(0,-48,8000,{.forwards = false});
    pros::delay(1000);
    topOuttake();
    pros::delay(5000);
    //go to + + match loader
    intake();
    chassis.moveToPoint(40,-45,2000);
    chassis.turnToHeading(0,1000);
    chassis.moveToPoint(40,48,2500);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(70, 48, 3000, {}, false);
    //match load + + and wiggle
    pros::delay(1000);
    loaderFork.retract();
    chassis.moveToPoint(-70,48,100,{.forwards = false});
    chassis.moveToPoint(70, 48, 5000, {}, false);
    //go to - + and score
    chassis.moveToPoint(40,48,2500, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    chassis.moveToPoint(25,60,1000);
    chassis.moveToPoint(-48,60,5000);
    chassis.waitUntilDone();
    distanceCode("West");
    pros::delay(200);
    chassis.moveToPoint(-48,48,2000);
    chassis.moveToPoint(48,48,2000, {.forwards = false}, true);
    pros::delay(750);
    topOuttake();
    //grab - + match loader
    loaderFork.extend();
    pros::delay(7000);
    intake();
    //wiggle
    chassis.moveToPoint(-75,50,2000);
    chassis.moveToPoint(75,50,150, {.forwards = false});
    chassis.moveToPoint(-75,50,2000);
    chassis.moveToPoint(75,50,150, {.forwards = false});
    chassis.moveToPoint(-75,50,7000);
    //score - +
    chassis.moveToPoint(75,50,7000, {.forwards = false},true);
    pros::delay(750);
    topOuttake();
    pros::delay(7000);
    //park and clear parking
    chassis.moveToPoint(-63,21,3000);
    chassis.moveToPoint(-64,0,2000,{.minSpeed = 127});
}


//match autons
void rightFull(){
  chassis.setPose(-48,-12,180);
  //grab loader and score 4 in long
  chassis.moveToPose(-48, -49,270, 3000);
  chassis.turnToHeading(270,2000);
  //go to loader
  loaderFork.extend();
  intake();
  pros::delay(3000);
  //tune timing
  chassis.moveToPoint(-78,-48,2000, {.forwards = false}, true);
  pros::delay(750);
  //score long
  topOuttake();
  loaderFork.retract();
  pros::delay(1000);
  //grab 3 field balls and score in bottom
  intake();
  chassis.moveToPose(-48, -49,270, 3000);
  chassis.turnToHeading(45,200, {.minSpeed = 72, .earlyExitRange = 8});
  chassis.moveToPoint(-9,-9,500, {}, true);
  pros::delay(200);
  loaderFork.extend();
  pros::delay(500);
  loaderFork.retract();
  chassis.moveToPoint(-17,-17, 300);
  bottomOuttake();
  pros::delay(350);
  //move 4 long balls into control
  chassis.moveToPose(-22,-36,315, 500, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
  Descore.extend();
  left_motors.set_brake_mode_all(pros::MotorBrake::hold);
  right_motors.set_brake_mode_all(pros::MotorBrake::hold);
  chassis.moveToPoint(-8,-36,200, {.forwards = false});
}


void leftFull(){
  chassis.setPose(-48,12,0);
  //grab balls from loader
  chassis.moveToPose(-48, 49,270, 3000);
  chassis.turnToHeading(270,2000);
  loaderFork.extend();
  chassis.moveToPoint(-70,49,500);
  //tune delay to only grab 3
  pros::delay(4500);
  //score 4 in long
  chassis.moveToPoint(70,49,1000,{.forwards = false},false);
  topOuttake();
  loaderFork.retract();
  pros::delay(4500);
  chassis.turnToHeading(135,200);
  chassis.moveToPoint(-17,17,500,{},true);
  //tune delay
  pros::delay(200);
  loaderFork.extend();
  pros::delay(500);
  loaderFork.retract();
  //turn around to mid score 3
  chassis.turnToHeading(315,400);
  chassis.moveToPoint(-9,9,200, {.forwards = false}, false);
  midOuttake();
  //go to defend
  chassis.moveToPose(-27.5,36,270, 750, {.minSpeed = 72});
  Descore.extend();
  left_motors.set_brake_mode_all(pros::MotorBrake::hold);
  right_motors.set_brake_mode_all(pros::MotorBrake::hold);
  chassis.moveToPoint(-11,36,400, {.forwards = false});
}


void rightQuick(){
  chassis.setPose(-48,-12,180);
  //grab loader and score 4 in long
  chassis.moveToPose(-48, -49,270, 3000);
  chassis.turnToHeading(270,2000);
  //go to loader
  loaderFork.extend();
  intake();
  pros::delay(3000);
  //tune timing
  chassis.moveToPoint(75,-48,2000, {.forwards = false}, true);
  pros::delay(750);
  //score long
  topOuttake();
  loaderFork.retract();
  pros::delay(1000);
  chassis.moveToPose(-48,-36,270,400,{.minSpeed = 72});
  Descore.extend();
  left_motors.set_brake_mode_all(pros::MotorBrake::hold);
  right_motors.set_brake_mode_all(pros::MotorBrake::hold);
  chassis.moveToPoint(-11,-36,200);

}


void leftQuick(){
  chassis.setPose(-48,12,0);
  //grab loader and score 4 in long
  chassis.moveToPose(-48, 49,270, 3000);
  chassis.turnToHeading(270,2000);
  //go to loader
  loaderFork.extend();
  intake();
  pros::delay(3000);
  //tune timing
  chassis.moveToPoint(75,48,2000, {.forwards = false}, true);
  pros::delay(750);
  //score long
  topOuttake();
  loaderFork.retract();
  pros::delay(1000);
  chassis.moveToPose(-48,36,270,400,{.minSpeed = 72});
  Descore.extend();
  left_motors.set_brake_mode_all(pros::MotorBrake::hold);
  right_motors.set_brake_mode_all(pros::MotorBrake::hold);
  chassis.moveToPoint(-11,36,200);
}


void soloAWP(){
  chassis.setPose(-48,5,90);
  //drop preload in mid
  chassis.moveToPoint(-15,5,500,{.forwards = false});
  midOuttake();
  pros::delay(200);
  intake();
  //grab 3
  chassis.moveToPose(-25,-25,180,500, {.minSpeed = 72}, true);
  pros::delay(100);
  loaderFork.extend();
  pros::delay(200);
  chassis.turnToHeading(45,200);
  loaderFork.retract();
  //score in bottom
  chassis.moveToPoint(-17,-17,500);
  pros::delay(200);
  bottomOuttake();
  pros::delay(400);
  //grab 3 from loader
  chassis.moveToPoint(-48,-48,500);
  chassis.turnToHeading(270,200);
  loaderFork.extend();
  intake();
  chassis.moveToPoint(-70,-48,400);
  pros::delay(500);
  //score in long
  chassis.moveToPoint(70,-48,400, {.forwards = false}, false);
  topOuttake();
}









//Testing autons
void drive24(){
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0,0,0);
  chassis.moveToPoint(0,24,5000);
}
void turn90(){
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0,0,0);
  chassis.turnToHeading(90,5000);
}

// void test360() {
//   chassis.setPose(0, 0, 0);
//   chassis.moveToPoint(0, -12, 1000, {.forwards = false});
//   chassis.turnToHeading(180, 5000,
//                         {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
//                          .maxSpeed = 70,
//                          .earlyExitRange = .00000000001});
//   chassis.turnToHeading(0, 5000,
//                         {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
//                          .maxSpeed = 70,
//                          .earlyExitRange = .00000000001});

//   {
//     {                                                  // Get sensor readings
//       double distNorth = dNorth.get_distance() / 25.4; // Convert to inches
//       double distNorthW = dSouth.get_distance() / 25.4;

//       // Expected sensor reading at (0,0,0) - when aligned with wall
//       double expectedDistance = 9.1;

//       // Calculate angle error (in degrees) -  sensors are facing Y axis now
//       // If dNorthW > dNorth, robot is rotated clockwise, theta error is
//       // negative
//       double theta_error_rad =
//           atan2((distNorthW - distNorth),
//                 13.75); // Still use horizontal separation for angle
//       double theta_error_deg = theta_error_rad * (180 / M_PI);

//       // Calculate Y position error (in inches) - sensors facing Y axis
//       double averageDistance = (distNorth + distNorthW) / 2.0;
//       double y_error = averageDistance - expectedDistance;

//       // Get current pose
//       lemlib::Pose current_pose = chassis.getPose();

//       // Calculate corrected pose - now correcting Y and Theta
//       double corrected_theta = current_pose.theta - theta_error_deg;
//       double corrected_y = current_pose.y - y_error;
//       double corrected_x = current_pose.x; // Keep X unchanged

//       // Set corrected pose
//       chassis.setPose(corrected_x, corrected_y, corrected_theta);

//       std::cout << "dNorth: " << distNorth << " inches, dNorthW: " << distNorthW
//                 << " inches" << std::endl;
//       std::cout << "Theta Error: " << theta_error_deg
//                 << " degrees, Y Error: " << y_error << " inches" << std::endl;
//       std::cout << "Corrected Pose: x=" << corrected_x << ", y=" << corrected_y
//                 << ", theta=" << corrected_theta << std::endl;
//     }
//   }
// }