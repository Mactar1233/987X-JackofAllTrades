#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"

pros::Motor catapult(1, pros::E_MOTOR_GEARSET_36, false,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Intake(11, pros::E_MOTOR_GEARSET_06, false,
                   pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIDigitalOut wingActuation('A');
pros::ADIDigitalOut blockerActuation('B');

bool climberLock = false;
bool blockerState = false;
int Rumblecount = 0;
bool drivermatchloading = false;

void setIntake(int speed) { Intake.move_voltage(speed * 120); }

void wingControl(bool state) { wingActuation.set_value(state); }

void matchLoad(bool matchLoading) {
  if (matchLoading == true) {
    catapult.move_voltage(10000);
  } else {
    catapult.move_voltage(0);
  }
}

void slapperControl() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
    drivermatchloading = !drivermatchloading;
    matchLoad(drivermatchloading);
  }
}

void intakeControl() {
  Intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) -
             master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) *
            100);
}

void wingTeleControl() {
    wingControl(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
  }

void blockerTeleControl(){
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
    blockerState = !blockerState;
  }

  blockerActuation.set_value(blockerState);
}