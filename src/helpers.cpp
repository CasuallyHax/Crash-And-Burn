#include "globals.hpp" 
#include "pros/misc.h"
#include "helpers.hpp"

int IntakeMode = 0;

void intake(){
    intakeMotor.move(-127);
    hoodMotor.move(-20);
}
void topOuttake(){
    intakeMotor.move(-127);
    hoodMotor.move(127);   
}
void bottomOuttake(){
    intakeMotor.move(127*.8);
    hoodMotor.move(-127);
}

void controllerCode(){
    
    //Intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        IntakeMode = 1;
    }
    if(IntakeMode == 1){
        intake();
    }

    if(controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L1)||controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L2)||controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1)){
        IntakeMode = 4;
    }

    if(IntakeMode == 4){
        intakeMotor.move(0);
        hoodMotor.move(0);
    }

    //Bottom
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        IntakeMode = 2;
    }
    if(IntakeMode == 2){
        bottomOuttake();
    }

    //Top
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        IntakeMode = 3;
    }
    if(IntakeMode == 3){
        topOuttake();
    }
    
    //Descore
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        Descore.toggle();
    }

    //LoaderFork
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        loaderFork.toggle();
    }

}

void pneumaticCode(){
    
}