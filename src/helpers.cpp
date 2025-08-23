#include "globals.hpp" // IWYU pragma: keep
#include "helpers.hpp"

//Preset scoring directions
void IntakeToBucket(){
    LoaderFork.retract();
    Intake_Bottom_11W.move(127);
    Intake_Middle_55W.move(127);
    Intake_Top_11W.move(-127);
    Bucket_55W.move(-127);
}

void TopScoring(){
    Intake_Bottom_11W.move(127);
    Intake_Middle_55W.move(-127);
    Intake_Top_11W.move(127);
    Bucket_55W.move(127);
}

void MiddleScoring(){
    Aligner.retract();
    LoaderFork.extend();
    Intake_Bottom_11W.move(127);
    Intake_Middle_55W.move(-127);
    Intake_Top_11W.move(-127);
    Bucket_55W.move(-127);
}

void BottomScoring(){
    Intake_Bottom_11W.move(-127);
    Intake_Middle_55W.move(-127);
    Intake_Top_11W.move(-127);
    Bucket_55W.move(-127);
}

