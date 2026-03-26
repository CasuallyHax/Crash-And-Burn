#include "robot/skills.h"
#include "api.h"
#include "globals.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include <random>

enum Wall {
  NORTH,
  SOUTH,
  EAST,
  WEST
};

void correctFromWall(
  Wall wall,
  float desiredDistance,   // inches from wall
  float wallCoord,         // field X or Y of that wall
  float timeout = 600
) {
  auto pose = chassis.getPose();
  float sensed = 0;
  float error = 0;

  switch (wall) {
    case NORTH:
      sensed = dNorth.get() / 25.4; // inches
      error = sensed - desiredDistance;
      pose.y = wallCoord - sensed;
      chassis.setPose(pose.x, pose.y, pose.theta);
      chassis.moveToPoint(pose.x, wallCoord - desiredDistance, timeout);
      break;

    case SOUTH:
      sensed = dSouth.get() / 25.4;
      error = sensed - desiredDistance;
      pose.y = wallCoord + sensed;
      chassis.setPose(pose.x, pose.y, pose.theta);
      chassis.moveToPoint(pose.x, wallCoord + desiredDistance, timeout);
      break;

    case EAST:
      sensed = dEast.get() / 25.4;
      error = sensed - desiredDistance;
      pose.x = wallCoord - sensed;
      chassis.setPose(pose.x, pose.y, pose.theta);
      chassis.moveToPoint(wallCoord - desiredDistance, pose.y, timeout);
      break;

    case WEST:
      sensed = dWest.get() / 25.4;
      error = sensed - desiredDistance;
      pose.x = wallCoord + sensed;
      chassis.setPose(pose.x, pose.y, pose.theta);
      chassis.moveToPoint(wallCoord + desiredDistance, pose.y, timeout);
      break;
  }
}

void correctNorth(float dist) {
  correctFromWall(NORTH, dist, 72.0);
}

void correctSouth(float dist) {
  correctFromWall(SOUTH, dist, -72.0);
}

void correctEast(float dist) {
  correctFromWall(EAST, dist, 72.0);
}

void correctWest(float dist) {
  correctFromWall(WEST, dist, -72.0);
}


void skills() {

}
