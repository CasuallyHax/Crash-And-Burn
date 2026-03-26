#include "globals.hpp"
#pragma once

void stallRecoveryTask(void*);

extern int distint;
extern float nDistCenter;
extern const float sDistCenter;
extern const float eDistCenter;
extern const float wDistCenter;
extern double seventy;
void distanceCode(std::string distance);

// Match specific routines
void match1(); // Match specific autonomous routine 1
void match2(); // Match specific autonomous routine 2
void test360();

void skills1();
void rightFull();
void leftFull();
void rightQuick();
void leftQuick();
void soloAWP();
void left7();
void right7();
void drive24();
void turn90();

extern const float negnegLoaderY;
extern const float posnegLoaderY;
extern const float posposLoaderY;
extern const float negposLoaderY;