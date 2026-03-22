#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot/auton.hpp"
#include "robot/monte.hpp"
#include "helpers.hpp"
#include <cmath>
#include <iostream>
#include <string>

using namespace lemlib;


void stallRecoveryTask(void*) {
  const int commandedVoltage = 12000;   // full power
  const int stallVelocity = 10;          // rpm threshold
  const int reverseTime = 300;           // ms
  const int checkDelay = 20;              // ms

  while (true) {
    intakeMotor.move_voltage(commandedVoltage);

    double actualVel = fabs(intakeMotor.get_actual_velocity());

    // Check for stall
    if (actualVel < stallVelocity &&
        fabs(intakeMotor.get_voltage()) > 1000) {

      // Reverse briefly
      intakeMotor.move_voltage(-commandedVoltage);
      pros::delay(reverseTime);

      // Resume forward
      intakeMotor.move_voltage(commandedVoltage);
    }

    pros::delay(checkDelay);
  }
}

int distint = 0;
float nDistCenter = 4.25;
const float sDistCenter = 1.5;
const float eDistCenter = 3.25;
const float wDistCenter = 4.495;

const float negnegLoaderY = -45.5;
const float posnegLoaderY = -46;
const float posposLoaderY = 48;
const float negposLoaderY = 48;

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
    // - - quadrant
    // set position to actual position measured by distance sensors 
    chassis.setPose(
    chassis.getPose().x, // use current x value, as we need to only adjust for the y of the match loader
    //convert to in, offset the the distance sensor to center of robot and calculate how far from the wall
    dNorth.get_distance()/25.4+nDistCenter-70, 
    chassis.getPose().theta); // keep the same theta
    chassis.waitUntilDone();
    if(chassis.getPose().y<negnegLoaderY-.5){ // Closer to the wall, need to back up
      chassis.moveToPoint(-48,-46.5,500,{.forwards = false});
    }else if(chassis.getPose().y>negnegLoaderY+.5){ // further from the wall, need to go forward
      chassis.moveToPoint(-48,-46.5,500);
    }
    break;
    
    case 2:
    //+-
    chassis.setPose(
    chassis.getPose().x,
    dSouth.get_distance()/25.4+sDistCenter-70,
    chassis.getPose().theta);
    chassis.waitUntilDone();
    pros::lcd::print(7, "sotuhPredict: %f", dSouth.get_distance()/25.4+sDistCenter-70); // s
    pros::delay(300);  
    if(chassis.getPose().y>posnegLoaderY+.5){ // further from the wall, need to backup
      chassis.moveToPoint(48,posnegLoaderY,500,{.forwards = false});
    }else if(chassis.getPose().y<posnegLoaderY-.5){ // closer to the wall, need to move forward
      chassis.moveToPoint(48,posnegLoaderY,500);
    }
    break;

    case 3:
    //++
    chassis.setPose(
    chassis.getPose().x,
    70-dNorth.get_distance()/25.4-nDistCenter,
    chassis.getPose().theta);
    if(chassis.getPose().y>posposLoaderY+.5){ // closer to the wall, need to backup
      chassis.moveToPoint(chassis.getPose().x,posposLoaderY,500,{.forwards = false});
    }else if(chassis.getPose().y<posposLoaderY-.5){ // further from the wall, need to move forward
      chassis.moveToPoint(chassis.getPose().x,posposLoaderY,500);
    }
    break;
    
    case 4:
    //-+
    chassis.setPose(
    chassis.getPose().x,
    70-dSouth.get_distance()/25.4-sDistCenter,
    chassis.getPose().theta);
    if(chassis.getPose().y<negposLoaderY-.5){ // further from the wall, need to back up
      chassis.moveToPoint(-48,negposLoaderY,500,{.forwards = false});
    }else if(chassis.getPose().y>negposLoaderY+.5){ // closer to the wall, need to move forward
      chassis.moveToPoint(-48,negposLoaderY,500);
    }
    break;
    }
  }
  
const int pibJiggle = 25;

//match autons
void skills1(){
    //Start touching red park
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Descore.extend();
    chassis.setPose(-48,-12,180);
    chassis.moveToPoint(-48, -46, 3000,{},false);
    chassis.waitUntilDone();
    pros::delay(200);
    distanceCode("--");
    pros::delay(50);
    chassis.turnToHeading(270,2000);
    //go to loader
    loaderFork.extend();
    intake();
    //wiggle in - - match loader
    pros::delay(125);
    for(int move1=0;move1<pibJiggle;move1++){
      chassis.moveToPoint(-75,-46.5,150);
      pros::delay(200);
    }
    intakeStop();
    //go to + - to score
    chassis.moveToPoint(-48,-50,3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    loaderFork.retract();
    chassis.moveToPoint(-27.5, -60, 3000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 8});
    chassis.moveToPoint(25, -60, 4000, {.forwards = false});
    chassis.moveToPoint(48,-44,1500, {.forwards = false});
    // //align against wall
    // chassis.moveToPoint(48,-100,300,{.forwards = false, .maxSpeed = 75});
    // chassis.swingToHeading(0,lemlib::DriveSide::LEFT,100);
    // pros::delay(100);
    chassis.turnToHeading(90,750);
    //go + - score
    chassis.moveToPoint(0,-48,3000,{.forwards = false}, true);
    pros::delay(1000);
    topOuttake();
    chassis.turnToHeading(90,1000,{.minSpeed = 40, .earlyExitRange = 10});
    chassis.setPose(chassis.getPose().x,dEast.get_distance()/25.4+eDistCenter-70,chassis.getPose().theta);
    pros::delay(650);
    loaderFork.extend();
    bottomOuttake();
    pros::delay(500);
    topOuttake();
    pros::delay(400);
    //grab + - match loader and wiggle
    intake();
    chassis.moveToPoint(48,posnegLoaderY,1700, {.maxSpeed= 55,.minSpeed = 40,.earlyExitRange = 5});  // match load +- quadrant
    for(int move1=0;move1<pibJiggle;move1++){
      chassis.moveToPoint(80,posnegLoaderY,150);
      pros::delay(200);
    }
    //score in + -
    chassis.moveToPoint(0,-50.5,3000,{.forwards = false});
    pros::delay(1000);
    topOuttake();
    chassis.turnToHeading(90,1000);
    chassis.setPose(chassis.getPose().x,dEast.get_distance()/25.4+eDistCenter-70,chassis.getPose().theta);
    pros::delay(650);
    bottomOuttake();
    pros::delay(500);
    topOuttake();
    pros::delay(300);
    //go to + + match loader
    intake();
    chassis.moveToPoint(35,-45,2000);
    chassis.moveToPoint(35,48,2000,{.minSpeed = 72, .earlyExitRange = 9});
    chassis.turnToHeading(0,1000);
    chassis.waitUntilDone();
    pros::delay(200);
    distanceCode("++");
    pros::delay(100);
    chassis.turnToHeading(90, 1000);
    //match load + + and wiggle
    chassis.moveToPoint(48,posposLoaderY,1700, {.maxSpeed= 85,.minSpeed = 40,.earlyExitRange = 5}); // go forwards a little, and then jiggle forwards
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
    pros::delay(200);
    chassis.turnToHeading(270,500);
    chassis.moveToPoint(0,48,2000, {.forwards = false}, true);
    pros::delay(1000);
    topOuttake();
    chassis.turnToHeading(270,1000);
    chassis.setPose(chassis.getPose().x,70-dEast.get_distance()/25.4-eDistCenter,chassis.getPose().theta);
    pros::delay(600);
    bottomOuttake();
    loaderFork.extend();
    pros::delay(300);
    topOuttake();
    pros::delay(200);
    intake();
    //wiggle
    chassis.moveToPoint(-48,negposLoaderY,1700, {.maxSpeed= 85,.minSpeed = 40,.earlyExitRange = 5});
    for(int move1=0;move1<pibJiggle;move1++){
      chassis.moveToPoint(-75,negposLoaderY,150);
      pros::delay(200);
    }
    pros::delay(500);
    //score - +
    chassis.moveToPoint(75,50,3000, {.forwards = false},true);
    pros::delay(200);
    loaderFork.retract();
    pros::delay(800);
    topOuttake();
    chassis.turnToHeading(270,1000);
    chassis.setPose(chassis.getPose().x,70-dEast.get_distance()/25.4-eDistCenter,chassis.getPose().theta);
    pros::delay(700);
    bottomOuttake();
    pros::delay(300);
    topOuttake();
    pros::delay(200);
    loaderFork.retract();
    //park and clear parking
    intake();
    chassis.moveToPose(-65,17,180,3000, {},false);
    loaderFork.extend();
    chassis.moveToPoint(-68,0,1300,{.minSpeed = 127});
}

const int pimpJiggle = 14;

//match autons
void rightFull(){
  chassis.setPose(-48,-12,180);
      Descore.extend();
    chassis.setPose(-48,-12,180);
    chassis.moveToPoint(-48, -46, 3000,{},false);
    chassis.waitUntilDone();
    pros::delay(200);
    distanceCode("--");
    pros::delay(100);
    chassis.turnToHeading(270,2000);
    //go to loader
    loaderFork.extend();
    intake();
    //wiggle in - - match loader
    pros::delay(125);
    for(int move1=0;move1<pimpJiggle;move1++){
      chassis.moveToPoint(-75,-46.5,150);
    pros::delay(200);
    }
  //grab loader and score 4 in long
  chassis.moveToPose(-48, -49,270, 3000);
  chassis.turnToHeading(270,2000);
  //go to loader
  loaderFork.extend();
  intake();
  pros::delay(3000);
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
  chassis.moveToPoint(-8,-36,200, {.forwards = false, .maxSpeed = 45});
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
  chassis.moveToPoint(-48,48,2000);
  chassis.turnToHeading(135,200);
  chassis.moveToPoint(-17,17,500,{},true);
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
  chassis.moveToPoint(-11,36,400, {.maxSpeed = 45});
}
const int pimpJiggle1 = 11;
void rightQuick(){
    chassis.setPose(-48,-12,180);
    chassis.moveToPoint(-48,-46.5,1500);
    chassis.turnToHeading(180,2000);
    chassis.waitUntilDone();
    pros::delay(50);
    distanceCode("--");
    chassis.turnToHeading(270,500);
    intake();
    loaderFork.extend();
    //wiggle in - - match loader
    pros::delay(125);
    for(int move1=0;move1<pimpJiggle1;move1++){
      chassis.moveToPoint(-75,-47,150);
    pros::delay(200);
    }
    //top score
    chassis.moveToPoint(48,-48,3200,{.forwards = false});
    pros::delay(900);
    topOuttake();
    // chassis.setPose(chassis.getPose().x, dWest.get_distance()+wDistCenter-70, chassis.getPose().theta);
    // pros::lcd::print(3, "westPredict: %f", dWest.get_distance()/25.4+wDistCenter-70); // w
    // pros::lcd::print(4, "distance w: %f", dWest.get_distance()); // w
    chassis.moveToPoint(-37,-39,5000);
    chassis.turnToHeading(270,500);
    //descore
    Descore.retract();
    left_motors.set_brake_mode_all(pros::MotorBrake::hold);
    right_motors.set_brake_mode_all(pros::MotorBrake::hold);
    chassis.moveToPoint(-3,-39,1000,{.forwards = false,.maxSpeed=55});
}


void leftQuick(){
  chassis.setPose(-48,12,0);
  //grab loader and score 4 in long
  chassis.moveToPose(-48, 49,270, 3000);
  chassis.turnToHeading(270,2000);
  //go to loader
  loaderFork.extend();
    intake();
    //wiggle in - - match loader
    pros::delay(125);
    for(int move1=0;move1<pimpJiggle;move1++){
      chassis.moveToPoint(-75,-46.5,150);
    pros::delay(200);
    }
  pros::delay(3000);
  chassis.moveToPoint(75,48,2000, {.forwards = false}, true);
  pros::delay(750);
  //score long
  topOuttake();
  loaderFork.retract();
  pros::delay(1000);
  chassis.moveToPose(-48,60,270,400,{.maxSpeed = 45});
  Descore.retract();
  left_motors.set_brake_mode_all(pros::MotorBrake::hold);
  right_motors.set_brake_mode_all(pros::MotorBrake::hold);
  chassis.moveToPoint(-8,60,200,{.forwards = false,.maxSpeed= 45});
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

void right7(){
    Descore.extend();
    chassis.setPose(-48,-12,180);
    chassis.moveToPoint(-48, -49, 3000);
    chassis.turnToHeading(45,200, {.minSpeed = 72, .earlyExitRange = 8});
    intake();
    chassis.moveToPoint(-9,-9,900, {}, true);
    pros::delay(700);
    loaderFork.extend();
    chassis.moveToPoint(-48,-46.5,1500);
    chassis.turnToHeading(180,2000);
    chassis.waitUntilDone();
    pros::delay(50);
    distanceCode("--");
    chassis.turnToHeading(270,500);
    //wiggle in - - match loader
    pros::delay(125);
    for(int move1=0;move1<pimpJiggle;move1++){
      chassis.moveToPoint(-75,-47,150);
    pros::delay(200);
    }
    //top score
    chassis.moveToPoint(48,-48,3200,{.forwards = false});
    pros::delay(900);
    topOuttake();
    // chassis.setPose(chassis.getPose().x, dWest.get_distance()+wDistCenter-70, chassis.getPose().theta);
    // pros::lcd::print(3, "westPredict: %f", dWest.get_distance()/25.4+wDistCenter-70); // w
    // pros::lcd::print(4, "distance w: %f", dWest.get_distance()); // w
    chassis.moveToPoint(-37,-36.5,5000);
    chassis.turnToHeading(270,500);
    //descore
    Descore.retract();
    left_motors.set_brake_mode_all(pros::MotorBrake::hold);
    right_motors.set_brake_mode_all(pros::MotorBrake::hold);
    chassis.moveToPoint(-8,-36.5,500,{.forwards = false,.maxSpeed=95});

}

void left7(){

    Descore.extend();
    chassis.setPose(-48,12,180);
    chassis.moveToPoint(-48, 49, 3000);
    chassis.turnToHeading(135,200, {.minSpeed = 72, .earlyExitRange = 8});
    intake();
    chassis.moveToPoint(-9,9,500, {}, true);
    pros::delay(200);
    loaderFork.extend();
    chassis.moveToPoint(-48, 46, 3000, {.forwards = false});
    chassis.turnToHeading(180,200);
    chassis.waitUntilDone();
    pros::delay(200);
    //Distance
    chassis.setPose(
    chassis.getPose().x,
    70-dSouth.get_distance()/25.4-sDistCenter,
    chassis.getPose().theta);
    chassis.waitUntilDone();
    if(chassis.getPose().y<posnegLoaderY-.5){ // Closer to the wall, need to back up
      chassis.moveToPoint(-48,posnegLoaderY,500,{.forwards = false});
    }else if(chassis.getPose().y>posnegLoaderY+.5){ // further from the wall, need to go forward
      chassis.moveToPoint(-48,posnegLoaderY,500);
    }
    pros::delay(100);
    chassis.turnToHeading(270,2000);
    //wiggle in - + match loader
    pros::delay(125);
    for(int move1=0;move1<pimpJiggle;move1++){
      chassis.moveToPoint(-75,negposLoaderY,150);
    pros::delay(200);
    }
    //top score & reset pose
    chassis.moveToPoint(48,48,5000,{.forwards = false});
    pros::delay(900);
    topOuttake();
    chassis.turnToHeading(90,1000);
    chassis.setPose(chassis.getPose().x,70-dEast.get_distance()/25.4-eDistCenter,chassis.getPose().theta);
    pros::delay(750);
    bottomOuttake();
    pros::delay(500);
    topOuttake();
    pros::delay(1500);
    chassis.moveToPoint(-48,36,5000);
    chassis.turnToHeading(270,500);
    //descore
    Descore.retract();
    left_motors.set_brake_mode_all(pros::MotorBrake::hold);
    right_motors.set_brake_mode_all(pros::MotorBrake::hold);
    chassis.moveToPoint(-8,60,500,{.forwards = false,.maxSpeed=45});




    Descore.extend();
    chassis.setPose(-48,12,90);
    chassis.moveToPose(-48, 49,270, 3000);
    chassis.turnToHeading(135,200, {.minSpeed = 72, .earlyExitRange = 8});
    chassis.moveToPoint(-9,9,500, {}, true);
    pros::delay(200);
    loaderFork.extend();
    chassis.moveToPoint(-48, 46, 3000);
    chassis.turnToHeading(180,200);
    chassis.waitUntilDone();
    pros::delay(200);
    
    pros::delay(100);
    chassis.turnToHeading(270,2000);
    //go to loader
    loaderFork.extend();
    intake();
    //wiggle in - - match loader
    pros::delay(125);
    for(int move1=0;move1<pimpJiggle;move1++){
      chassis.moveToPoint(-75,46.5,150);
    pros::delay(200);
    }
    //top score & reset pose
    chassis.moveToPoint(48,48,5000,{.forwards = false});
    pros::delay(900);
    topOuttake();
    chassis.turnToHeading(90,1000);
    chassis.setPose(chassis.getPose().x,dEast.get_distance()/25.4+eDistCenter-70,chassis.getPose().theta);
    pros::delay(750);
    bottomOuttake();
    pros::delay(500);
    topOuttake();
    pros::delay(1500);
    chassis.moveToPoint(48,60,5000);
    chassis.turnToHeading(270,500);
    //descore
    Descore.retract();
    chassis.moveToPoint(8,60,500,{.forwards = false,.maxSpeed=45});

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
// Drives an L-shaped path near two walls then returns to origin.
// At each waypoint, pauses 2s showing odom vs MCL vs diff on the LCD.
// Place robot at (-48, -48, 0) — bottom-left corner, facing north.
void testMCL(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(-48, -48, 0);
    MCL::localizer.init(-48, -48, 0);
    pros::lcd::print(0, "MCL TEST: starting");
    pros::delay(1000);

    // Leg 1: north along west wall — South + West sensors close
    chassis.moveToPoint(-48, 0, 5000);
    chassis.waitUntilDone();
    {
        lemlib::Pose o = chassis.getPose();
        lemlib::Pose m = MCL::localizer.getPose();
        pros::lcd::print(0, "LEG1 odom  x:%.1f y:%.1f t:%.1f", o.x, o.y, o.theta);
        pros::lcd::print(1, "LEG1 mcl   x:%.1f y:%.1f t:%.1f", m.x, m.y, m.theta);
        pros::lcd::print(2, "LEG1 diff  x:%.1f y:%.1f t:%.1f", m.x-o.x, m.y-o.y, m.theta-o.theta);
    }
    pros::delay(2000);

    // Leg 2: turn east, drive along horizontal — South sensor active
    chassis.turnToHeading(90, 3000);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 0, 5000);
    chassis.waitUntilDone();
    {
        lemlib::Pose o = chassis.getPose();
        lemlib::Pose m = MCL::localizer.getPose();
        pros::lcd::print(3, "LEG2 odom  x:%.1f y:%.1f t:%.1f", o.x, o.y, o.theta);
        pros::lcd::print(4, "LEG2 mcl   x:%.1f y:%.1f t:%.1f", m.x, m.y, m.theta);
        pros::lcd::print(5, "LEG2 diff  x:%.1f y:%.1f t:%.1f", m.x-o.x, m.y-o.y, m.theta-o.theta);
    }
    pros::delay(2000);

    // Apply MCL correction and show final corrected pose
    MCL::localizer.applyCorrection();
    {
        lemlib::Pose o = chassis.getPose();
        pros::lcd::print(6, "CORRECTED  x:%.1f y:%.1f t:%.1f", o.x, o.y, o.theta);
        pros::lcd::print(7, "DONE");
    }
}

void telemetry(){
  while(true){
    lemlib::Pose odom = chassis.getPose();
    lemlib::Pose mcl  = MCL::localizer.getPose();

    float dN = dNorth.get_distance() / 25.4f;
    float dS = dSouth.get_distance() / 25.4f;
    float dE = dEast.get_distance()  / 25.4f;
    float dW = dWest.get_distance()  / 25.4f;

    pros::lcd::print(0, "ODOM x:%.1f y:%.1f t:%.1f", odom.x, odom.y, odom.theta);
    pros::lcd::print(1, "MCL  x:%.1f y:%.1f t:%.1f", mcl.x,  mcl.y,  mcl.theta);
    pros::lcd::print(2, "DIFF x:%.1f y:%.1f t:%.1f", mcl.x-odom.x, mcl.y-odom.y, mcl.theta-odom.theta);
    pros::lcd::print(3, "N:%.1f S:%.1f E:%.1f W:%.1f", dN, dS, dE, dW);
    pros::lcd::print(4, "N:%s S:%s E:%s W:%s",
      dN < MCL::MAX_RANGE_IN ? "ON " : "off",
      dS < MCL::MAX_RANGE_IN ? "ON " : "off",
      dE < MCL::MAX_RANGE_IN ? "ON " : "off",
      dW < MCL::MAX_RANGE_IN ? "ON " : "off");
    pros::delay(100);
  }
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