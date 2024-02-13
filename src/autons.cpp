#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/vision.h"
#include <fstream>
#include <mutex>

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

const int DRIVE_SPEED =
    110; // This is 110/127 (around 87% of max speed).  We don't suggest making
         // this 127. If this is 127 and the robot tries to heading correct,
         // it's only correcting by making one side slower.  When this is 87%,
         // it's correcting by making one side faster and one side slower,
         // giving better heading correction.
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier
// game objects, or with lifts up vs down. If the objects are light or the cog
// doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 50, 3, 250, 7, 250, 500); //100, 3, 500, 7, 500, 500
  chassis.set_exit_condition(chassis.swing_exit, 30, 3, 200, 7, 200, 500); // 100, 3, 500, 7, 500, 500
  chassis.set_exit_condition(chassis.drive_exit, 30, 100, 150, 150, 200, 500);//80, 50, 300, 150, 500, 500
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a
  // slew at the start of drive motions for slew, only enable it when the drive
  // distance is greater then the slew distance + a few inches

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining
  // distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot
                             // will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining
  // distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot
                             // will go the remaining distance at 40 speed
  chassis.wait_drive();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive

  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
}

// . . .
// Make your own autonomous functions here!
// . . .

void fourpointfive() {
  // Intake First Ball
  setIntake(100);
  chassis.set_drive_pid(15, 90);
  chassis.wait_drive();
  pros::delay(200);
  // Back Out to Match Loader
  chassis.set_drive_pid(-43.3, 45, true);
  chassis.wait_until(-20);
  chassis.set_max_speed(70);
  chassis.wait_drive();
  setIntake(0);
  // Turn next to the Bar
  chassis.set_swing_pid(ez::RIGHT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();
  // Back up to pull triball out of match loader
  chassis.set_drive_pid(-5, DRIVE_SPEED);
  chassis.wait_drive();
  wingControl(true);
  // pull ball out
  chassis.set_drive_pid(-11.5, 127);
  chassis.wait_drive();
  // Turn to goal
  chassis.set_swing_pid(ez::RIGHT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();
  // Push into Goal and drive forward
  chassis.set_drive_pid(-15.5, 127);
  chassis.wait_drive();
  chassis.set_drive_pid(11, DRIVE_SPEED);
  chassis.wait_drive();
  wingControl(false);
  chassis.set_turn_pid( 270,  TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(15, 127);
  setIntake(-100);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(-7, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-13.5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(16,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(314, TURN_SPEED);
  chassis.wait_drive();
  setIntake(100);
  chassis.set_drive_pid(60.5, 80);
  chassis.wait_until(40);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING,360, 70);
  chassis.wait_drive();
  chassis.set_drive_pid(5, DRIVE_SPEED);
  chassis.wait_drive();
  wingControl(true);
  chassis.set_drive_pid(-35, 127);

}

void fourpointfivefixed() {
  // Intake First Ball
  setIntake(100);
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(200);
  // Back Out to Match Loader
  chassis.set_drive_pid(-38, 45, true);
  chassis.wait_until(-10);
  chassis.set_max_speed(70);
  chassis.wait_drive();
  // Turn next to the Bar
  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
  BwingControl(true); 

  // Back up to pull triball out of match loader

  chassis.set_drive_pid(-24, 90);
  chassis.wait_drive();
  BwingControl(false);
  chassis.set_drive_pid(5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-10,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  setIntake(-100);
  chassis.set_drive_pid(20, 127);
  chassis.wait_drive();
  chassis.set_drive_pid(-16, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(23, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(54, 127);
  
  
  chassis.wait_drive();
  setIntake(100);
  chassis.set_turn_pid(157, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(15, DRIVE_SPEED);
  setIntake(-100);
  chassis.wait_drive();
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(17, DRIVE_SPEED);
  chassis.wait_drive();
  setIntake(100);
  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(1.2, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(180, DRIVE_SPEED);
  chassis.wait_drive();
  wingControl(true);
  setIntake(-100);
  chassis.set_drive_pid(38, 127);

}

void allianceStealD(){
  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, 70 );  
  chassis.wait_drive();
  chassis.set_drive_pid( 10.5, 127, true);
  setIntake(-100);
  chassis.wait_drive();
  setIntake(0);
  chassis.set_drive_pid(-5, DRIVE_SPEED);
  chassis.wait_drive();
  setIntake(100);
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-11, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(12, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0 , TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(22, 80);
  chassis.wait_drive();
  setIntake(0);
  chassis.set_drive_pid(-17, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  setIntake(-100);
  chassis.set_drive_pid(20, 127);


  

}

void awpStealD(){
  setIntake(100);
  blockerControl(true);
pros::delay(50);
blockerControl(false);
  chassis.set_drive_pid(51, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED); 
  chassis.wait_drive();
      setIntake(-100);

  chassis.set_drive_pid( 13, 127);
  blockerControl(true);
  chassis.wait_drive();
  chassis.set_drive_pid(-9, DRIVE_SPEED);
  chassis.wait_drive();
  blockerControl(false);
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
    setIntake(100);
  chassis.set_drive_pid(-11, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(11, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0 , TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(17, 80);
  chassis.wait_drive();
  chassis.set_drive_pid(-16, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(19, 127);
    setIntake(-100);

 chassis.wait_drive();
  setIntake(0);
  chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-36.5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(ez:: LEFT_SWING, -30, SWING_SPEED);
  chassis.wait_drive();
      wingControl(true);


  chassis.set_drive_pid(-5.5, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, -90, SWING_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(6, 127);
  chassis.wait_drive();
    wingControl(false);

    chassis.set_drive_pid(-35, DRIVE_SPEED);
chassis.wait_drive();
  
  blockerControl(true);

 


  

}

void skills(){
    wingControl(true);
    pros::delay(500);
    wingControl(false);
    chassis.wait_drive();
  setIntake(-100);
  chassis.set_turn_pid(-45,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 70, SWING_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-10, DRIVE_SPEED);
  BwingControl(true);
  chassis.wait_drive();
  BwingControl(false);
  chassis.set_turn_pid( 180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-8, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();
  //goes thorugh lane
  chassis.set_drive_pid(-80, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(225, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-30, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-10,DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-13, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(7, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING, 315, SWING_SPEED);
  chassis.wait_drive();




}

void threeballAWP() {
  // Intake First Ball
  setIntake(100);
  chassis.set_drive_pid(15, 90);
  chassis.wait_drive();
  pros::delay(200);
  // Back Out to Match Loader
  chassis.set_drive_pid(-43, 45, true);
  chassis.wait_until(-10);
  chassis.set_max_speed(70);
  chassis.wait_drive();
  setIntake(0);
  // Turn next to the Bar
  chassis.set_swing_pid(ez::LEFT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();
  // Back up to pull triball out of match loader
  chassis.set_drive_pid(-5, DRIVE_SPEED);
  chassis.wait_drive();
  wingControl(true);
  // pull ball out
  chassis.set_drive_pid(-11.5, 127);
  chassis.wait_drive();
  // Turn to goal
  chassis.set_swing_pid(ez::LEFT_SWING, -90, SWING_SPEED);
  chassis.wait_drive();
  // Push into Goal and drive forward
  chassis.set_drive_pid(-15.5, 127);
  chassis.wait_drive();
  chassis.set_drive_pid(11, DRIVE_SPEED);
  chassis.wait_drive();
  wingControl(false);
  chassis.set_turn_pid( 90,  TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(15, 127);
  setIntake(-100);
  chassis.wait_drive();
  pros::delay(200);
  chassis.set_drive_pid(-7, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-14, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(16.8,DRIVE_SPEED);
  chassis.wait_drive();
  setIntake(0);
chassis.set_turn_pid(-45, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(20, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(180, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(-38, DRIVE_SPEED);
}

void sixballauton() {
blockerControl(true);
pros::delay(50);
blockerControl(false);
wingControl(true);
pros::delay(350);
wingControl(false);
//pointing at middle ball
 chassis.set_drive_pid(64,DRIVE_SPEED);
 chassis.wait_drive();
 chassis.set_turn_pid(-45, SWING_SPEED);
 setIntake(100);
 chassis.wait_drive();
 chassis.set_angle(0);
 chassis.set_drive_pid(3, 127);
 chassis.wait_drive();
 //reverse to smack both ball in 
 chassis.set_turn_pid(180, TURN_SPEED);
 chassis.wait_drive();
 //turn robot to outake
 wingControl(true);
 setIntake(-50);
 chassis.set_drive_pid(29, DRIVE_SPEED);
 chassis.wait_drive();
 wingControl(false);
chassis.set_turn_pid(-30, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(33, DRIVE_SPEED);
setIntake(100);
chassis.wait_drive();
setIntake(0);
chassis.set_turn_pid(-150, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(36.5, DRIVE_SPEED);
setIntake(-60);

chassis.wait_drive();
setIntake(0);
chassis.set_turn_pid(-90, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(19.5, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(0, TURN_SPEED);
chassis.wait_drive();
setIntake(100);
chassis.set_drive_pid(36.5, 100);
chassis.wait_drive();
chassis.set_drive_pid(-35.5, 127);
chassis.wait_drive();
chassis.set_turn_pid(135, TURN_SPEED);
chassis.wait_drive();
wingControl(true);
chassis.set_drive_pid(20,DRIVE_SPEED);
chassis.wait_drive();
chassis.set_swing_pid(ez::RIGHT_SWING,90, TURN_SPEED);
wingControl(false);
setIntake(-100);
chassis.wait_drive();
chassis.set_drive_pid(20, 127);





}

void threeballsave(){
  blockerControl(true);
pros::delay(50);
blockerControl(false);
wingControl(true);
pros::delay(350);
wingControl(false);
//pointing at middle ball
 chassis.set_drive_pid(64,DRIVE_SPEED);
 chassis.wait_drive();
 chassis.set_turn_pid(-45, SWING_SPEED);
 setIntake(100);
 chassis.wait_drive();
 chassis.set_angle(0);
 chassis.set_drive_pid(3, 127);
 chassis.wait_drive();
 //reverse to smack both ball in 
 chassis.set_turn_pid(180, TURN_SPEED);
 chassis.wait_drive();
 //turn robot to outake
 wingControl(true);
 setIntake(-70);
 chassis.set_drive_pid(35, DRIVE_SPEED);
 chassis.wait_drive();
 wingControl(false);
 chassis.set_drive_pid(-30, DRIVE_SPEED);
 chassis.wait_drive();
 chassis.set_turn_pid(220, TURN_SPEED);
 chassis.wait_drive();
 setIntake(100);
 chassis.set_drive_pid(45, DRIVE_SPEED);
 chassis.wait_drive();
 chassis.set_swing_pid(ez::RIGHT_SWING, 90, SWING_SPEED);
 chassis.wait_drive();
 setIntake(-100);
 chassis.set_drive_pid(18, 127);
 chassis.wait_drive();
 chassis.set_drive_pid(-10, DRIVE_SPEED);
 chassis.wait_drive();
  chassis.set_drive_pid(18, 127);
 chassis.wait_drive();
 chassis.set_drive_pid(-10, DRIVE_SPEED);
 chassis.wait_drive();
 
}

void mactar2Ball(){
   setIntake(100);
  blockerControl(true);
pros::delay(50);
blockerControl(false);
  chassis.set_drive_pid(51, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED); 
  chassis.wait_drive();
      setIntake(-100);

  chassis.set_drive_pid( 13, 127);
  blockerControl(true);
  chassis.wait_drive();
  chassis.set_drive_pid(-9, DRIVE_SPEED);
  chassis.wait_drive();
  blockerControl(false);
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
    setIntake(100);
  chassis.set_drive_pid(-11, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(11, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0 , TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(17, 80);
  chassis.wait_drive();
  chassis.set_drive_pid(-16, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_until(80);
            setIntake(-100);

  chassis.wait_drive();
  chassis.set_drive_pid(13, 127);
  chassis.wait_drive();
  chassis.set_drive_pid(-5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(225, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(39, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 315, SWING_SPEED);
  chassis.wait_drive();
    wingControl(true);

  chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
 chassis.wait_drive();
    chassis.set_drive_pid(3, DRIVE_SPEED);
chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, -135, SWING_SPEED);
  chassis.wait_drive();

    wingControl(false);
    chassis.set_turn_pid(270, DRIVE_SPEED);
    chassis.wait_drive();



  chassis.set_drive_pid(-30, DRIVE_SPEED);
  chassis.wait_drive();

 
}

void DangerousDefensiveAWP(){

  wingControl(true);
  pros::delay(500);
  wingControl(false);
  chassis.set_drive_pid(43.5, 127);
  chassis.wait_drive();
  chassis.set_turn_pid(90, DRIVE_SPEED);
  wingControl(true);
  chassis.wait_drive();
  chassis.set_drive_pid(33, 127);
  chassis.wait_drive();
  wingControl(false);
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-57, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-20, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(180, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(315, TURN_SPEED);
  chassis.wait_drive();
  BwingControl(true);
  chassis.set_drive_pid(-25, DRIVE_SPEED);
  chassis.wait_drive();
  BwingControl(false);
  chassis.set_drive_pid(3, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(34.5, DRIVE_SPEED);


}

void SafedefensiveAWP(){
  setIntake(100);
    blockerControl(true);
    pros::delay(200);
    blockerControl(false);
  chassis.wait_drive();
  chassis.set_turn_pid(-45,TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
  setIntake(-100);
  chassis.set_drive_pid(-7, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-12.5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(15, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(315, TURN_SPEED);
  chassis.wait_drive();
  BwingControl(true);
  chassis.set_drive_pid(-18, DRIVE_SPEED);
  chassis.wait_drive();
  BwingControl(false);
  chassis.set_drive_pid(5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-9, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(36.5, DRIVE_SPEED);
  chassis.wait_drive();
  blockerControl(true);
}

void riskyAWPDefense(){
wingControl(true);  
pros::delay(150);
setIntake(100);
  chassis.wait_drive();
  wingControl(false);
  chassis.set_turn_pid(15, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(52, 127);
  chassis.wait_drive();
  chassis.set_drive_pid(-52, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(34, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-5, DRIVE_SPEED);
  chassis.wait_drive();
 chassis.set_drive_pid(13, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
  BwingControl(true);
  chassis.set_drive_pid(-20, DRIVE_SPEED);
  chassis.wait_drive();
  BwingControl(false);
  chassis.set_drive_pid(3, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-7, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(3, DRIVE_SPEED);
  setIntake(-100);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(36, DRIVE_SPEED);
  chassis.wait_drive();
  blockerControl(true);
  




}

void SixBallR () {
  setIntake(100);
chassis.set_turn_pid(-35, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(63, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(90, TURN_SPEED);
chassis.wait_drive();
wingControl(true);
setIntake(-100);
chassis.set_drive_pid(36, DRIVE_SPEED);
chassis.wait_drive();
wingControl(false);
chassis.set_drive_pid(-3, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(230, TURN_SPEED);
chassis.wait_drive();
setIntake(100);
chassis.set_drive_pid(38,DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(300, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(-35, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(-315, TURN_SPEED);
setIntake(-100);
chassis.wait_drive();
chassis.set_turn_pid(0, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(-15, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(270, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(18, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_turn_pid(90, TURN_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(10, DRIVE_SPEED);


}
void rigoExample(){

    chassis.set_drive_pid(2, DRIVE_SPEED);
    chassis.wait_drive();
    chassis.set_drive_pid(-2, DRIVE_SPEED);
    setIntake(100);
    chassis.wait_drive();
    chassis.set_turn_pid(90, TURN_SPEED);
    chassis.wait_drive();
    chassis.set_turn_pid(80, TURN_SPEED);
    chassis.wait_drive();
    chassis.set_swing_pid( ez::RIGHT_SWING, 145, SWING_SPEED);
    chassis.wait_drive();


    chassis.set_drive_pid(4, 50);
    chassis.wait_drive();
    chassis.set_drive_pid(16, 127);


    chassis.set_drive_pid(20, 50);
    chassis.wait_until(4);
    chassis.set_max_speed(127);


    chassis.set_drive_pid(50, DRIVE_SPEED);
    chassis.wait_until(10);
    wingControl(true); 
    chassis.wait_until(25);
    wingControl(false);
    chassis.wait_drive();


}