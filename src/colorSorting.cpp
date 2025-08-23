#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "helpers.hpp" // IWYU pragma: keep
#include "globals.hpp" // IWYU pragma: keep
#include "colorSorting.hpp"

// ---------- HELPERS ----------
bool in_range(double x, double a, double b) {
    return x >= a && x <= b;
}

Color classify_sample(double hue) {
    if (in_range(hue, 0, 30) || in_range(hue, 330, 360)) return Color::RED;
    if (in_range(hue, 200, 250)) return Color::BLUE;
    return Color::UNKNOWN;
}

Color classify_block() {
    int prox = ColorSorter.get_proximity();
    if (prox < PROX_MIN) return Color::UNKNOWN;

    int red = 0, blue = 0;
    for (int i = 0; i < VOTE_FRAMES; i++) {
        double hue = ColorSorter.get_hue();
        Color c = classify_sample(hue);
        if (c == Color::RED) red++;
        if (c == Color::BLUE) blue++;
        pros::delay(10);
    }

    if (red >= VOTE_FRAMES / 2) return Color::RED;
    if (blue >= VOTE_FRAMES / 2) return Color::BLUE;
    return Color::UNKNOWN;
}



void colorSorting(){
    Color detected = classify_block(); // check current block
    while(true){
        if (detected == TeamColor) {
            IntakeToBucket(); // handle friendly block
            } else if (detected != Color::UNKNOWN) {
                TopScoring(); // handle enemy block
                pros::delay(2500);
            } else {
                IntakeToBucket(); // no block detected
            }

        pros::delay(25);
    }
}