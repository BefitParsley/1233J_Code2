#include <type_traits>
#include "liblvgl/misc/lv_area.h"
#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 80;
const int TURN_SPEED = 75;
const int SWING_SPEED = 100;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(15.0, 2.5, 137.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(2.8, 0.10, 30.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Red Left
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  
  chassis.pid_drive_set(33.00_in, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  mogo.set(true);
  
  intake3.move(127);
  chassis.pid_drive_set(-3_in, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(13.8_in, 85);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.4_in, 90);
  chassis.pid_wait();
  intake.move(127);
  intake3.move(127);
  chassis.pid_wait();
  
  pros::delay(1050);
  chassis.pid_turn_set(269_deg, TURN_SPEED);

  chassis.pid_drive_set(-28.45_in, 50);
 
  chassis.pid_wait();
  pros::delay(10);
  intake.move(127);
  intake3.move(127);
  intake2.move(127);
  pros::delay(2000);

  mogo.set(false);
  intake2.move(0);
  chassis.pid_drive_set(22.5_in, 60);
  chassis.pid_wait();
  chassis.pid_turn_set(130_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(30_in, 80);
  chassis.pid_wait();
  chassis.pid_turn_set(315_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24.85_in, 50);
  chassis.pid_wait();
  pros::delay(10);
  intake2.move(80);


  /*intake2.move(-40);
  intake.move(127);
  intake3.move(127);

  chassis.pid_drive_set(34.75_in, 55, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-107_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-15.2_in,DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  intake2.move(65);
  pros::delay(1100);

  chassis.pid_drive_set(46_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();*/
}

///
// Red Right
///
void turn_example() {
 chassis.pid_drive_set(32.2_in, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();


  mogo.set(true);
  intake.move(127);
  intake3.move(127);
  pros::delay(400);
  
 chassis.pid_drive_set(-3_in, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(11.8_in, 85);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.8_in, 90);
   chassis.pid_wait();
  chassis.pid_turn_set(-271_deg, TURN_SPEED);
  chassis.pid_wait();
  
  pros::delay(950);

  chassis.pid_drive_set(-27.7_in, 55);
  chassis.pid_wait();
  pros::delay(10);
  intake.move(127);
  intake3.move(127);
  intake2.move(127);
  pros::delay(2000);

  mogo.set(false);
 intake2.move(0);
 chassis.pid_drive_set(19_in, 60);
 chassis.pid_wait();
 chassis.pid_turn_set(-135_deg, TURN_SPEED);
 chassis.pid_wait();
 chassis.pid_drive_set(29.5_in, 65);
 chassis.pid_wait();
 intake.move(-90);
 intake3.move(-90);
 intake2.move(-90);
 chassis.pid_drive_set(17_in, 55);
 chassis.pid_wait();
 chassis.pid_turn_set(-140_deg, TURN_SPEED);
 chassis.pid_wait();
 pros::delay(4000);
}

///
// Solo
///
void drive_and_turn() {
  

  //low goal
  intake3.move(127);
  chassis.pid_drive_set(35.1_in, 90);
  chassis.pid_wait();
  intake3.move(0);
  chassis.pid_turn_set(-94_deg, 90);
  chassis.pid_wait_quick_chain();
  intake3.move(-127);
  intake.move(-127);
  pros::delay(300);
  chassis.pid_drive_set(11.7_in, 75);
  chassis.pid_wait_quick_chain();
  pros::delay(200);

 //mid goal
  chassis.pid_drive_set(-12.9_in, 80);
  chassis.pid_wait();
  chassis.pid_turn_set(-144_deg, 85);
  chassis.pid_wait_quick_chain();
  intake2.move(0);
  intake3.move(127);
  intake.move(127);
  chassis.pid_drive_set(25_in, 75);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(16_in, 65);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-185_deg, 85);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-14.3_in, 85);
  chassis.pid_wait_quick_chain();
  intake3.move(127);
  intake.move(127);
  intake2.move(80);
  pros::delay(1000);

  //matchload 
  intake2.move(0);
  chassis.pid_drive_set(44.65_in, 85);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-235_deg, 85);
  chassis.pid_wait();
  
  intake2.move(0);
  mogo.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(21.1_in, 70);
  chassis.pid_wait();
   chassis.pid_drive_set(-0.6_in, 80);
   chassis.pid_wait();
  chassis.pid_turn_set(-235_deg, 85);
  intake.move(127);
  intake3.move(127);
  pros::delay(900);
  
  
  //long goal pt 1
  chassis.pid_drive_set(-29.85_in, 90);
  chassis.pid_wait_quick_chain();
  pros::delay(10);
  intake.move(127);
  intake3.move(127);
  intake2.move(127);
  pros::delay(2000);

  //matchload clear/set up for driver
  chassis.pid_drive_set(29.25_in, 90);
}

///
// Skills 
///
void wait_until_change_speed() {
 chassis.pid_drive_set(33.4_in, 70, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();


  mogo.set(true);
  
  intake3.move(127);
  chassis.pid_drive_set(-3_in, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(11.8_in, 85);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.8_in, 90);
  chassis.pid_wait();
  intake.move(127);
  intake3.move(127);
  chassis.pid_wait();
  
  pros::delay(1200);
  //Matchload 1

  chassis.pid_turn_set(269_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-28.6_in, 50);
 
  chassis.pid_wait();
  pros::delay(10);
  intake.move(127);
  intake3.move(127);
  intake2.move(127);
  pros::delay(2000);
  //Score 1

  intake2.move(-60);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(30.45_in, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.8_in, 90);
  chassis.pid_wait();
  pros::delay(1450);
  //Matchload 2

  chassis.pid_turn_set(269_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-29.25_in, 50);
  chassis.pid_wait();

  pros::delay(10);
  intake.move(127);
  intake3.move(127);
  intake2.move(127);
  pros::delay(1800);
  //Score 2

  mogo.set(false);

  chassis.pid_drive_set(7_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 70);
  chassis.pid_wait();

  intake3.move(0);
  intake.move(0);
  intake2.move(0);

  chassis.pid_drive_set(75_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(-180_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-16_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-11.35_in, 70);
  chassis.pid_wait();
  //Move to Loader 2

  intake3.move(127);
  intake.move(127);

  mogo.set(true);
  pros::delay(10);
  chassis.pid_drive_set(30.5_in, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.8_in, 90);
  chassis.pid_wait();
  pros::delay(1300);
  //Matchload 3

  chassis.pid_turn_set(89_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-29.4_in, 50);
  chassis.pid_wait();

  pros::delay(10);
  intake2.move(127);
  pros::delay(2000);
  //Score 3

  intake2.move(-60);
  chassis.pid_drive_set(30.5_in, 75);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.8_in, 90);
  chassis.pid_wait();
  pros::delay(1450);
  //Matchload 4

  chassis.pid_turn_set(89_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-29.4_in, 50);
  chassis.pid_wait();

  pros::delay(10);
  intake2.move(127);
  pros::delay(2000);
  //Score 4

  

  mogo.set(false);
  chassis.pid_drive_set(11_in, 80);
  chassis.pid_wait(); 
  chassis.pid_turn_set(-180_deg, 80);
  chassis.pid_wait(); 
  chassis.pid_drive_set(15.8_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 80);
  chassis.pid_wait(); 
  chassis.pid_drive_set(97_in, 75);
  chassis.pid_wait();

  intake3.move(-127);
  intake.move(-127);
  intake2.move(-127);
  //Move to Park



  chassis.pid_turn_set(-160_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(50_in, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 90);
  chassis.pid_wait();
  chassis.pid_turn_set(-180_deg, 80);
  chassis.pid_wait();
  chassis.pid_drive_set(4_in, 90);
  chassis.pid_wait();
 

  


}

///
// Elims Right
///
void swing_example() {
  chassis.pid_drive_set(9.25_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  intake3.move(127);
  intake.move(127);
  chassis.pid_drive_set(30_in, 45);
  chassis.pid_wait();
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();
  intake3.move(0);
  intake.move(0);
  chassis.pid_drive_set(-36_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_set(91_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-18.15_in, 70);
  chassis.pid_wait();
  intake3.move(127); 
  intake.move(127);
  intake2.move(127);
  pros::delay(1500);
  intake2.move(0);
  mogo.set(true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(30.25_in, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-.25_in, 70);
  chassis.pid_wait();
  pros::delay(1100);
  chassis.pid_drive_set(-30_in, 70);
  chassis.pid_wait();
  intake2.move(127);
  pros::delay(1800);
  chassis.pid_drive_set(30_in, 70);
  
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .