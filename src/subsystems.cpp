#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

pros::Motor catapult(11, pros::E_MOTOR_GEARSET_36, true,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Intake1(8, pros::E_MOTOR_GEARSET_18, true,
                   pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Intake2(3, pros::E_MOTOR_GEARSET_18, false,
                   pros::E_MOTOR_ENCODER_DEGREES);    

pros::ADIDigitalOut wingActuation('B');
pros::ADIDigitalOut blockerActuation('D');
pros::ADIDigitalOut chomperactuation('E');
pros::ADIDigitalOut BwingActuation('A');

pros::Motor_Group Intake({Intake1, Intake2});

bool climberLock = false;
bool blockerState = false;
bool bWingState = false;
int Rumblecount = 0;
bool drivermatchloading = false;

void setIntake(int speed) { Intake.move_voltage(speed * 120); }

void wingControl(bool state) { wingActuation.set_value(state); }

void BwingControl(bool state){ BwingActuation.set_value(state); }

void blockerControl(bool state) { blockerActuation.set_value(state); }

void matchLoad(bool matchLoading, bool skills) {
  if (matchLoading == true) {
    catapult.move_voltage(12500);
    if(skills == true){
      catapult.move_voltage(12500);
    }
  } else {
    catapult.move_voltage(0);
  }
}

void slapperControl() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN  )) {
    drivermatchloading = !drivermatchloading;
    matchLoad(drivermatchloading, false);
  }
}

void intakeControl() {
  setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) -
             master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) *
            100);
}

void wingTeleControl() {
    wingControl(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
  }

void BwingTeleControl(){

BwingControl(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
}
void blockerTeleControl(){
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
    blockerState = !blockerState;
  }

  blockerActuation.set_value(blockerState);
}

void chomperTelecontrol(){
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
      chomperactuation.set_value(true);

  }
}