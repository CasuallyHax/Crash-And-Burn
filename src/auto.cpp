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
float nDistCenter = 5.25;
const float sDistCenter = 2.5;
const float eDistCenter = 4.5;
const float wDistCenter = 6.625;
double seventy = 70;
void distanceCode(std::string distance){

    int counter=1;
  if(distance == "North"){
    distint = 1;
  } else if(distance == "South"){
    distint = 2;
  } else if (distance == "East"){
    distint = 3;
  } else if (distance == "West"){
    distint = 4;}
  switch(distint){
    case 1:
    //facing north
    chassis.setPose(
    70-dEast.get_distance()/25.4-eDistCenter,
    70-dNorth.get_distance()/25.4-nDistCenter,
    chassis.getPose().theta);
    break;
    
    case 2:
    //facing south
    chassis.setPose(
    dEast.get_distance()/25.4+eDistCenter-70,
    dNorth.get_distance()/25.4+nDistCenter-70,
    chassis.getPose().theta);
    break;

    case 3:
    //facing east
    chassis.setPose(
    70-dNorth.get_distance()/25.4-nDistCenter,
    dEast.get_distance()/25.4+eDistCenter-70,
    chassis.getPose().theta);
    while(1){
    pros::lcd::print(3, "counter: %d", counter++); // x
    pros::lcd::print(4, "x: %f", 70-dNorth.get_distance()/25.4-nDistCenter); // x
    pros::lcd::print(5, "y: %f", 70-dNorth.get_distance()/25.4-nDistCenter); // y
    pros::delay(50);}
    break;
    
    case 4:
    //facing west
    chassis.setPose(
    dNorth.get_distance()/25.4+nDistCenter-70,
    70-dEast.get_distance()/25.4-eDistCenter,
    chassis.getPose().theta);
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

void test360() {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -12, 1000, {.forwards = false});
  chassis.turnToHeading(180, 5000,
                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
                         .maxSpeed = 70,
                         .earlyExitRange = .00000000001});
  chassis.turnToHeading(0, 5000,
                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
                         .maxSpeed = 70,
                         .earlyExitRange = .00000000001});

  {
    {                                                  // Get sensor readings
      double distNorth = dNorth.get_distance() / 25.4; // Convert to inches
      double distNorthW = dSouth.get_distance() / 25.4;

      // Expected sensor reading at (0,0,0) - when aligned with wall
      double expectedDistance = 9.1;

      // Calculate angle error (in degrees) -  sensors are facing Y axis now
      // If dNorthW > dNorth, robot is rotated clockwise, theta error is
      // negative
      double theta_error_rad =
          atan2((distNorthW - distNorth),
                13.75); // Still use horizontal separation for angle
      double theta_error_deg = theta_error_rad * (180 / M_PI);

      // Calculate Y position error (in inches) - sensors facing Y axis
      double averageDistance = (distNorth + distNorthW) / 2.0;
      double y_error = averageDistance - expectedDistance;

      // Get current pose
      lemlib::Pose current_pose = chassis.getPose();

      // Calculate corrected pose - now correcting Y and Theta
      double corrected_theta = current_pose.theta - theta_error_deg;
      double corrected_y = current_pose.y - y_error;
      double corrected_x = current_pose.x; // Keep X unchanged

      // Set corrected pose
      chassis.setPose(corrected_x, corrected_y, corrected_theta);

      std::cout << "dNorth: " << distNorth << " inches, dNorthW: " << distNorthW
                << " inches" << std::endl;
      std::cout << "Theta Error: " << theta_error_deg
                << " degrees, Y Error: " << y_error << " inches" << std::endl;
      std::cout << "Corrected Pose: x=" << corrected_x << ", y=" << corrected_y
                << ", theta=" << corrected_theta << std::endl;
    }
  }
}

// void hooks_score(int degrees, int direction) {
//   hooks.move_relative(degrees, 600 * direction);
//   hooks.brake();
// }

// void hooks_on(int speed) {
//   if (speed == 0) {
//     hooks.brake();
//     return;
//   }

//   hooks.move_velocity(600);
// }

// void hooks_off() { hooks_on(0); }

// void Auton1() {
//   // Autonomous winpoint blue positive side / red positive side

//   // score on alliance stake

//   chassis.setPose(-60, -12, 0);
//   chassis.moveToPose(-60, 0, 0, 5000);
//   chassis.turnToHeading(90, 1000);
//   chassis.moveToPoint(-65, 0, 1000, {.forwards = false});
//   hooks_score(2000, 1);

//   // pick up ring and score

//   chassis.setPose(-62, 0, 90, false);
//   hooks_on(600);
//   chassis.moveToPose(-24, -48, 135, 2700, {}, false);
//   hooks_off();

//   clamp.toggle();
//   chassis.turnToHeading(180, 2000);
//   chassis.moveToPoint(-24, -22, 5000, {.forwards = false, .maxSpeed = 25},
//                       false);
//   clamp.toggle();

//   pros::delay(500);
//   hooks_score(1000, 1);

//   chassis.turnToHeading(0, 1000);
//   chassis.moveToPoint(-20, -2, 5000, {.forwards = true, .maxSpeed = 40}, false);
// }

// void Auton2() {

//   pros::delay(5000);
//   // Autonomous winpoint blue negative side / red negative side

//   // score on alliance stake

//   chassis.setPose(-60, 24, 180);
//   chassis.moveToPose(-60, 0, 180, 5000);
//   chassis.turnToHeading(90, 1000);
//   chassis.moveToPoint(-65, 0, 1000, {.forwards = false});
//   hooks_score(2000, 1);

//   // pick up ring and score

//   chassis.setPose(-62, 0, 90, false);
//   hooks_on(600);
//   chassis.moveToPose(-24, 48, 45, 2700, {}, false);
//   hooks_off();

//   clamp.toggle();
//   chassis.turnToHeading(0, 2000);
//   chassis.moveToPoint(-24, 22, 5000, {.forwards = false, .maxSpeed = 25},
//                       false);
//   clamp.toggle();

//   pros::delay(500);
//   hooks_score(1000, 1);

//   chassis.turnToHeading(180, 1000);
//   chassis.moveToPoint(-20, 2, 5000, {.forwards = true}, false);
// }

// void Auton3() {
//   chassis.setPose(0, 0, 0, false);
//   chassis.moveToPose(0, -36, 0, 2700, {.forwards = false, .maxSpeed = 70},
//                      false);
//   clamp.extend();
//   hooks.move_velocity(600);
// }

// void Auton5() {
//   // Skills challenge autonomous

//   // Chassis position: coordinate from the back of the drivetrain
//   // Chassis heading: front hooks is direction

//   // Step 1. We start under the red alliance stake.
//   // With the preloaded ring, we will score on the stake using our wall stake
//   // mechanism.

//   chassis.setPose(-165, 0, 90, false);
//   hooks_score(1000, 1);
//   // -- TODO: Score on the red alliance stake

//   // Step 2. We will go to pick up the top left mobile goal
//   // with our clamp facing into the mobile goal.

//   chassis.turnToHeading(180, 5000, {}, false);

//   chassis.moveToPoint(-120, 60, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   clamp.toggle();

//   // Step 3. We will go and score the 6 rings around the mobile goal onto our
//   // robot. This will take a lot of precise coding and movement to nail
//   // autonomously

//   hooks_score(1000, 1);

//   // -- Score bottom right ring (1)

//   chassis.turnToHeading(90, 5000, {}, false);

//   chassis.moveToPoint(-60, 60, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Score second top ring (2)

//   chassis.turnToHeading(0, 5000, {}, false);

//   chassis.moveToPoint(-60, 120, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Score center top ring (3)

//   chassis.turnToHeading(90, 5000, {}, false);

//   chassis.moveToPoint(0, 150, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Score corner center ring (4)

//   chassis.turnToHeading(270, 5000, {}, false);

//   chassis.moveToPoint(-120, 120, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Score corner back left ring (5)

//   chassis.moveToPoint(-150, 120, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Score corner top ring (6)

//   chassis.moveToPoint(-120, 150, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // Step 4. We will go and put the fully scored out mobile goal into the top
//   // right corner to double its points.

//   chassis.moveToPose(-168, -168, 135, 5000, {.forwards = false}, false);
//   pros::delay(200);

//   clamp.toggle();

//   // Step 5. We will go to the center, and pick up the center ring on our robot.
//   // This will later be used to score on the bottom right mobile goal.

//   chassis.moveToPose(0, 0, 0, 5000, {.forwards = true}, false);
//   pros::delay(500);
//   hooks_score(1000, 1);

//   // Step 6. We will pick up the bottom right's mobile goal to score more rings
//   // onto.

//   chassis.moveToPose(-120, 60, 45, 5000, {.forwards = false}, false);

//   clamp.toggle();

//   // Step 7. We will pick up all of the rings in the bottom right corner.
//   // This will required high precision and a well-tuned autonomous to accomplish
//   // quickly.

//   // -- Pick up the top right ring (2)

//   hooks_score(1000, 1);

//   chassis.moveToPose(-60, -60, 135, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Pick up the bottom right ring (3)

//   chassis.moveToPose(-60, -120, 180, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Pick up the middle ring (4)
//   chassis.moveToPose(-120, -120, 270, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Pick up a ring (5)
//   chassis.moveToPose(-150, -120, 270, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // -- Pick up last ring (6)
//   chassis.moveToPose(-120, -150, 135, 5000, {.forwards = true}, false);
//   pros::delay(200);

//   // Step 8. We will put the mobile goal into the positive corner at the bottom
//   // right. This will double all of the points on our current mobile goal.

//   chassis.moveToPose(-166, -166, 45, 5000, {.forwards = false}, false);

//   // Step 9. Move to the center line, and pick up a ring, then turn around and
//   // score it on the high stakes.

//   chassis.moveToPose(0, -150, 90, 5000, {.forwards = true}, false);
//   // TODO: SCORE RING ON HIGH STAKE

//   // Step 10. We will go to the center bar and hang
//   chassis.moveToPose(-25, -40, 45, 5000, {.forwards = true}, false);
//   // TODO: TOGGLE HANG MECHANISM
// }

// void match1() {

//   chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
//   chassis.setPose(-62.4, 40.5, 270);
//   chassis.moveToPoint(-45, 40.5, 1000);
//   chassis.turnToPoint(-31.8, 29.4, 1000, {.forwards = false}, false);
//   chassis.moveToPoint(-31.8, 29.4, 1000, {.forwards = false}, false);

//   clamp.extend();

//   chassis.turnToPoint(-23.3, 48.8, 1000, {}, false);
//   hooks.move_velocity(600);
//   chassis.moveToPoint(-20.3, 50.8, 1000, {.maxSpeed = 50}, false);

//   chassis.turnToPoint(-67.1, 67, 1000);
//   chassis.moveToPoint(-67.1, 67, 1000);

//   pros::delay(3000);

//   chassis.moveToPose(-49, -23, 180, 3000);
// }

// void match2() {
//   // red neg

//   chassis.setPose(0, 0, 0, false);
//   chassis.moveToPose(0, -36, 0, 2700, {.forwards = false, .maxSpeed = 70},
//                      false);
//   clamp.extend();
//   hooks.move_velocity(600);

//   pros::delay(6000);

//   chassis.turnToHeading(90, 1000);
//   chassis.moveToPoint(0, 40, 1000);
// }
