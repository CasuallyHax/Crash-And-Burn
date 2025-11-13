#include "globals.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "helpers.hpp"

int IntakeMode = 0;

//Preset scoring directions
void IntakeToBucket(){
    Intake_Bottom_11W.move(127);
    Intake_Middle_55W.move(127);
    Intake_Top_11W.move(-127);
    Bucket_55W.move(0);
    Intake_Top_Front_55W.move(-127);
    Intake_Top_Back_55W.move(127);
    Agitator_55W.move(-127);
    colorSorting("Blue");
}

void TopScoring(){
    Intake_Bottom_11W.move(127);
    Intake_Middle_55W.move(127);
    Intake_Top_11W.move(-127);
    Bucket_55W.move(-127);
    Intake_Top_Front_55W.move(127);
    Intake_Top_Back_55W.move(-127);
    Agitator_55W.move(127);
}

void MiddleScoring(){
    Intake_Bottom_11W.move(127);
    Intake_Middle_55W.move(127);
    Intake_Top_11W.move(127);
    Bucket_55W.move(-127);
    Agitator_55W.move(127);
}

void BottomScoring(){
    Intake_Bottom_11W.move(-127);
    Intake_Middle_55W.move(-127);
    Intake_Top_11W.move(-127);
    Bucket_55W.move(-127);
    Agitator_55W.move(127);
}


void colorSorting(std::string goodColor){
    if(goodColor == "Red"){
        if(colorSorter.get_hue() >= 200 && colorSorter.get_hue() <= 240) {
            
            pros::delay(100);
            MiddleScoring();
            pros::delay(200);
            Intake_Bottom_11W.move(127);
            Intake_Middle_55W.move(127);
            Intake_Top_11W.move(-127);
            Bucket_55W.move(-127);
            Intake_Top_Front_55W.move(-127);
        }
    } if(goodColor == "Blue"){
            if(colorSorter.get_hue() >= 340 && colorSorter.get_hue() <= 360) {
                pros::delay(   100);
                MiddleScoring();
                pros::delay(200);
                Intake_Bottom_11W.move(127);
                Intake_Middle_55W.move(127);
                Intake_Top_11W.move(-127);
                Bucket_55W.move(-127);
                Intake_Top_Front_55W.move(-127);
        }

}
}

void controllerCode(){
    
    //Intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        IntakeMode = 1;
    }
    if(IntakeMode == 1){
        Hood.retract();
        IntakeToBucket();
    }

    if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L1)||controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L2)||controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R2)||controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1)){
        IntakeMode = 5;
    }

    //Bottom
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        IntakeMode = 2;
    }
    if(IntakeMode == 2){
        BottomScoring();
    }

    //Middle
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        IntakeMode = 3;
    }
    if(IntakeMode == 3){
        MiddleScoring();
    }

    //Top
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        IntakeMode = 4;
    }
    if(IntakeMode == 4){
        Hood.extend();
        TopScoring();
    }
    
    
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            LoaderFork.retract();
    }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        
        LoaderFork.extend();
    }
if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            LoaderFork.retract();
    }
 
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        
        LoaderFork.extend();
    }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    IntakeMode = 5;

        }
        
        if(IntakeMode == 5){
            Intake_Bottom_11W.move(0);
            Intake_Middle_55W.move(0);
            Intake_Top_11W.move(0);
            Bucket_55W.move(0);
            Intake_Top_Front_55W.move(0);
            Intake_Top_Back_55W.move(0);
            Agitator_55W.move(0);
        }
}

void pneumaticCode(){
    
}