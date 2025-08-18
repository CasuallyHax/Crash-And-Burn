#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "helpers.hpp"
#include "globals.hpp" // IWYU pragma: keep

// ---------- CONSTANTS ----------
constexpr int PROX_MIN = 30;
constexpr int VOTE_FRAMES = 8;

// ---------- FUNCTION DECLARATIONS ----------
Color classify_sample(double hue);
Color classify_block();
bool in_range(double x, double a, double b);


void colorSorting();