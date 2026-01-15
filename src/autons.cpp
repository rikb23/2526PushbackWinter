#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 100;
const int TURN_SPEED = 110;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
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
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
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

void medScore(int speed) {
   intake.move(-1 * speed);
            topintake.move(speed);
             small.set(false);
             med.set(false);
}

void longScore (int speed) {
     intake.move(-1 * speed);
            topintake.move(speed);
              med.set(true);
             small.set(false);
}

void ballLock(int speed) {
    intake.move(-1 * speed);
            topintake.move(speed);
              med.set(false);
             small.set(true);
        }


void initial_matchload() {
    // go to matchload
  chassis.pid_odom_set({{46_in, 22_in}, fwd, 90});
  chassis.pid_wait();
  // turn facing 180 deg
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  matchload.set(true);
  chassis.pid_wait();
  pros::delay(500);



  // drop matchload + turn on intake
  intake.move(-1*110);
  topintake.move(0);
  backintake.move(-1 * 110);
  chassis.pid_wait();

  chassis.pid_drive_set(7_in, 120, true);
  chassis.pid_wait();

  // go forward
  pros::delay(750);
  // go backward to approx 48 , 24 i think
  chassis.pid_odom_set({{46_in, 24_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  matchload.set(false);

  chassis.pid_wait();
}

void sev_twoGoal_blue() {
  // intitial position (x,y,90 deg) // x parallel to field wall, y perpendicular
  chassis.odom_xyt_set(15.5_in, 22_in, 90_deg);

  initial_matchload();

  chassis.pid_turn_set({20_in, 52_in}, fwd,  TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{36_in, 34_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{20_in, 52_in}, fwd, DRIVE_SPEED/2});
  chassis.pid_wait();
  // wait till blocks are in basket
  pros::delay(1500);
  intake.move(0);
  topintake.move(0);
  backintake.move(0);
  pros::delay(50);
  intake.move(-1 * 100);
  topintake.move(100);
  backintake.move(-100);
  pros::delay(200);
intake.move(0);
  topintake.move(0);
  backintake.move(0);
  pros::delay(50);
  intake.move(-1 * 100);
  topintake.move(100);
  backintake.move(-100);
  pros::delay(200);
  intake.move(0);
  topintake.move(0);
  backintake.move(0); 
  // score 1 or 2 in middle goal
  chassis.pid_odom_set({{11.3_in, 59.3_in}, fwd, DRIVE_SPEED/2});
  chassis.pid_wait();
  intake.move(100);
  topintake.move(100);
  backintake.move(100);
  pros::delay(800);
  intake.move(0);
  topintake.move(0);
  backintake.move(0);
  // align to long goal
  chassis.pid_odom_set({{46_in, 24_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{45_in, 42_in}, fwd, DRIVE_SPEED/2});
  chassis.pid_wait();
  // score rest of blocks 
  intake.move(-127);
  topintake.move(-127);
  backintake.move(127);
  pros::delay(100);
  intake.move(0);
  topintake.move(0);
  backintake.move(0);
  pros::delay(50);
   intake.move(-127);
  topintake.move(-127);
  backintake.move(127);
  pros::delay(100);
  intake.move(0);
  topintake.move(0);
  backintake.move(0);
  pros::delay(50); intake.move(-127);
  topintake.move(-127);
  backintake.move(127);
  pros::delay(100);
  intake.move(0);
  topintake.move(0);
  backintake.move(0);
  pros::delay(50);
  intake.move(-127);
  topintake.move(-127);
  backintake.move(127);
    pros::delay(5000);
  // if time left set up to clear matchload or push teamate off the thing

}

void sev_twoGoal_red() {
  chassis.odom_y_flip();
 chassis.odom_x_flip();
 chassis.odom_theta_flip();
 chassis.odom_y_direction_get();  // True = down is positive Y, False = up is positive Y
 chassis.odom_x_direction_get();  // True = left is positive X, False = right is positive X
 chassis.odom_theta_direction_get(); // True = positive is counterclockwise, False = positive is counterclockwise


  sev_twoGoal_blue();
}
void solo_awp_blue() {
  // intitial position (x,y,90 deg) // x parallel to field wall, y perpendicular
  chassis.odom_xyt_set(15.5_in, 22_in, 90_deg);
 
  initial_matchload();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 40_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  intake.move(127);
  topintake.move(127);
  backintake.move(-127);
  pros::delay(2000);

  chassis.pid_odom_set({{48_in, 24_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_turn_set({24_in, 48_in}, fwd,  TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{30_in, 54_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{20_in, 64_in}, fwd, DRIVE_SPEED/2});
  chassis.pid_wait();
  // wait till blocks are in basket
  pros::delay(2000);
  // score 1 or 2 in middle goal
  chassis.pid_odom_set({{18_in, 66_in}, fwd, DRIVE_SPEED/2});
  chassis.pid_wait();
  intake.move(-1 * 80);
  topintake.move(80);
  backintake.move(80);
  pros::delay(500);
  intake.move(127);
  topintake.move(127);
  backintake.move(-127);

}

int sevenSpeed = 90;
int skillsSpeed = 110;
void skillsOldOld() {
  chassis.odom_xyt_set(15.5_in, 22_in, 90_deg);
  // go to matchload
    // go to matchload
  chassis.pid_odom_set({{46_in, 22_in}, fwd, 90});
  chassis.pid_wait();
  // turn facing 180 deg
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  matchload.set(true);
  chassis.pid_wait();
  pros::delay(500);



  // drop matchload + turn on intake
  intake.move(-1*110);
  topintake.move(0);
  backintake.move(-1 * 110);
  chassis.pid_wait();

  chassis.pid_drive_set(7_in, 120, true);
  chassis.pid_wait();

  chassis.pid_drive_set(3_in, 70, true);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

   pros::delay(1500);

  chassis.pid_drive_set(4_in, 50, true);
  chassis.pid_wait();

  // go forward
 
  // go backward to approx 48 , 24 i think
  chassis.pid_odom_set({{46_in, 24_in}, rev, skillsSpeed});
  chassis.pid_wait();
  matchload.set(false);

  chassis.pid_wait();
  // odom cool movement to cross the field
  chassis.pid_odom_set({{{60_in, 45_in}, rev, skillsSpeed},
                      {{60_in, 88_in}, rev, skillsSpeed},
                      {{47_in, 120_in}, rev, skillsSpeed},
                      {{47_in, 103_in}, fwd, skillsSpeed}},
                      true);
  chassis.pid_wait();
  intake.move(-127);
  topintake.move(-127);
  backintake.move(127);
  pros::delay(3000);
  // pick up 2nd matchload

  chassis.pid_odom_set({{43_in, 123.5_in,}, rev, skillsSpeed});
  chassis.pid_wait();

  intake.move(0);
  topintake.move(0);
  backintake.move(0);

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  matchload.set(true);
  chassis.pid_wait();
  pros::delay(500);



  // drop matchload + turn on intake
  intake.move(-1*110);
  topintake.move(0);
  backintake.move(-1 * 110);
  chassis.pid_wait();

  chassis.pid_drive_set(7_in, 120, true);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();
  
    chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

    chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

   chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

    pros::delay(2000);

  chassis.pid_drive_set(4_in, 50, true);
  chassis.pid_wait();

  // go forward
  // go backward to approx 48 , 24 i think
  chassis.pid_odom_set({{46_in, 120_in}, rev, skillsSpeed});
  chassis.pid_wait();
  matchload.set(false);

  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{46_in, 104_in}, fwd, skillsSpeed});
  chassis.pid_wait();

  intake.move(-127);
  topintake.move(-127);
  backintake.move(127);
  pros::delay(3000);

  chassis.pid_drive_set(-12_in, 50, true);
  chassis.pid_wait();

    chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-10_in, 120_in}, rev, skillsSpeed});
  chassis.pid_wait();


  chassis.pid_odom_set({{-49_in, 125_in}, rev, skillsSpeed});
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

    matchload.set(true);
    pros::delay(500);
  // drop matchload + turn on intake
  intake.move(-1*110);
  topintake.move(0);
  backintake.move(-1 * 110);
  chassis.pid_wait();

  chassis.pid_drive_set(7_in, 120, true);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();
  
    chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

    chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

   chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

    pros::delay(2000);

  chassis.pid_drive_set(4_in, 50, true);
  chassis.pid_wait();

  // go forward
  // go backward to approx 48 , 24 i think
  chassis.pid_odom_set({{-51_in, 123_in}, rev, skillsSpeed});
  chassis.pid_wait();
  matchload.set(false);
  pros::delay(2000);

  chassis.pid_odom_set({{{-63_in, 130_in}, rev, skillsSpeed},
                      {{-65_in, 55_in}, rev, skillsSpeed},
                      {{-46_in, 24_in}, rev, skillsSpeed},
                      {{-49_in, 45_in}, fwd, skillsSpeed}},
                      true);
  chassis.pid_wait();

  chassis.pid_wait();
  intake.move(-127);
  topintake.move(-127);
  backintake.move(127);
      intake.move(-127);
  topintake.move(127);
  backintake.move(-127);

   chassis.pid_drive_set(-12_in, 70, true);
  chassis.pid_wait();

  chassis.pid_turn_set(140_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set({{-15_in, 6_in}, fwd, skillsSpeed});
  chassis.pid_wait();

  
  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, SWING_SPEED, 15);
  chassis.pid_wait();


    intake.move(-127);
  topintake.move(127);
  backintake.move(-127);
  
   chassis.pid_drive_set(12_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(-18_in, 50, true);
  chassis.pid_wait();

  chassis.pid_turn_set(100_deg, TURN_SPEED);
  chassis.pid_wait();

    chassis.pid_drive_set(24_in, 127, true);
  chassis.pid_wait();














}

void skills() {
      chassis.odom_xyt_set(19.5_in, 7.5_in, 0_deg);


    chassis.pid_odom_set({{{19.5_in, 36_in}, fwd, skillsSpeed},
                        { {28_in, 50_in}, fwd, 40}},
                         true);
    chassis.pid_wait();
    pros::delay(500);

      chassis.pid_odom_set({{52_in, 24_in}, rev, skillsSpeed});
  chassis.pid_wait();

  
     intake.move(-127);
    topintake.move(127);
      small.set(false);
      med.set(false);

      chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

      matchload.set(true);
    pros::delay(300);


  chassis.pid_drive_set(11.5_in, 120, true);
  chassis.pid_wait();

  pros::delay(100);

   chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

     chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();


/*  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();*/


 pros::delay(2000);

    chassis.pid_drive_set(4_in, 50, true);
  chassis.pid_wait();

  
     chassis.pid_odom_set({{52_in, 48_in,180_deg}, rev, skillsSpeed});
  chassis.pid_wait();

    matchload.set(false);
    chassis.pid_wait();
    
intake.move(-127);
            topintake.move(127);
              med.set(true);
             small.set(false);
    pros::delay(5000);

         chassis.pid_drive_set(10_in, 70, true);
  chassis.pid_wait();

      intake.move(-127);
      topintake.move(127);
        small.set(false);
        med.set(false);

  
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

   

      chassis.pid_odom_set({{-44_in, 30_in}, fwd, skillsSpeed});
  chassis.pid_wait();

      chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

      matchload.set(true);
    chassis.pid_wait();
    pros::delay(500);


  chassis.pid_drive_set(16.5_in, 120, true);
  chassis.pid_wait();

  chassis.pid_drive_set(3_in, 70, true);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

/*  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(1_in, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1_in, 70, true);
  chassis.pid_wait();*/

   chassis.pid_drive_set(4_in, 50, true);
  chassis.pid_wait();

 pros::delay(2000);

     chassis.pid_odom_set({{-43_in, 48_in}, rev, skillsSpeed});
  chassis.pid_wait();
  
    matchload.set(false);
    chassis.pid_wait();

  intake.move(-127);
  topintake.move(127);
    med.set(true);
    small.set(false);

      pros::delay(5000);

      intake.move(-127);
  topintake.move(127);

chassis.pid_drive_set(10_in, 127, true);
  chassis.pid_wait();

        chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

    chassis.pid_odom_set({{3_in, 36_in}, fwd, skillsSpeed});
  chassis.pid_wait();

         chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in, 127, true);
  chassis.pid_wait();

   chassis.pid_drive_set(80_in, 127, true);
  chassis.pid_wait();


   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();

   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();


   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();


   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();


   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();


   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();


   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();


   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();

   chassis.pid_drive_set(-2_in, 127, true);
  chassis.pid_wait();

    chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();
 





}
void pid() {

}
// chassis.odom_xyt_set(19.5_in, 7_in, 0_deg);
void sevenBall() {
  


    intake.move(-1*127);
    topintake.move(0);
    backintake.move(-1 * 127);
    chassis.pid_odom_set({{{19.5_in, 36_in}, fwd, sevenSpeed},
                        { {28_in, 50_in}, fwd, 40}},
                         true);
    chassis.pid_wait();
    pros::delay(1100);

    chassis.pid_turn_set({50_in, 24_in}, fwd,  TURN_SPEED);
    chassis.pid_wait();

      chassis.pid_odom_set({{52.3_in, 24_in}, fwd, sevenSpeed});
  chassis.pid_wait();

    matchload.set(true);
    pros::delay(500);

      chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();


     chassis.pid_drive_set(7_in, 120, true);
  chassis.pid_wait();

  chassis.pid_drive_set(5_in, 70, true);
  chassis.pid_wait();

 

  pros::delay(90);


  
     chassis.pid_odom_set({{54_in, 24_in}, rev, sevenSpeed});
  chassis.pid_wait();

        chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

    matchload.set(false);
    chassis.pid_wait();
    pros::delay(500); 


         chassis.pid_drive_set(15_in, 70);
  chassis.pid_wait();

  intake.move(27);
    topintake.move(27);
    backintake.move(27);
    
     chassis.pid_drive_set(4_in, 30);
  chassis.pid_wait();

    intake.move(-127);
    topintake.move(-127);
    backintake.move(127);

    pros::delay(1000);

            intake.move(27);
            topintake.move(27);
            backintake.move(27);

      pros::delay(25);

          intake.move(-127);
    topintake.move(-127);
    backintake.move(127);

    pros::delay(1000);

    intake.move(0);
    topintake.move(0);
    backintake.move(0);

           chassis.pid_drive_set(1_in, 30);
  chassis.pid_wait();

          intake.move(-127);
    topintake.move(-127);
    backintake.move(127);

    pros::delay(1000);

    intake.move(0);
    topintake.move(0);
    backintake.move(0);

           chassis.pid_drive_set(1_in, 30);
  chassis.pid_wait();

          intake.move(-127);
    topintake.move(-127);
    backintake.move(127);

    pros::delay(1000);

    intake.move(0);
    topintake.move(0);
    backintake.move(0);

      pros::delay(25);

          intake.move(-127);
    topintake.move(-127);
    backintake.move(127);

    pros::delay(1000);

    intake.move(0);
    topintake.move(0);
    backintake.move(0);

      pros::delay(25);

          intake.move(-127);
    topintake.move(-127);
    backintake.move(127);

    pros::delay(1000);

    intake.move(0);
    topintake.move(0);
    backintake.move(0);

      pros::delay(25);


          intake.move(-127);
    topintake.move(-127);
    backintake.move(127);

        pros::delay(3000);










}

void sevenBallHigh() {
  chassis.odom_xyt_set(-19.5_in, 7_in, 0_deg);
    chassis.odom_x_flip();
  chassis.odom_theta_flip();
  sevenBall();
}

void sevenBallLow() {
    chassis.odom_xyt_set(19.5_in, 7_in, 0_deg);

  sevenBall();
}
void park() {
    intake.move(-1*110);
  topintake.move(0);
  backintake.move(-1 * 110);
     chassis.pid_drive_set(-17_in, 70, true);
  chassis.pid_wait();
    chassis.pid_drive_set(60_in, 100, true);
  chassis.pid_wait();
  pros::delay(5000);
}

void sawp() {

  chassis.odom_xyt_set(15.5_in, 22_in, 90_deg);

   matchload.set(true);
   intake.move(-127);
            topintake.move(127);
              med.set(false);
             small.set(false);


   chassis.pid_odom_set({{48_in, 22_in}, fwd, 115});
  chassis.pid_wait();
  // turn facing 180 deg
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

      chassis.pid_drive_set(7.5_in, 110, true);
  chassis.pid_wait_quick_chain();

   chassis.pid_drive_set(-5_in, 127, true);
  chassis.pid_wait_quick_chain();



  chassis.pid_odom_set({{50_in, 50_in}, rev, 110});
  chassis.pid_wait();

         intake.move(-127);
            topintake.move(127);
              med.set(true);
             small.set(false);

matchload.set(false);

  pros::delay(1400);

     chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait_quick_chain();

   med.set(false);
    small.set(false);
  chassis.pid_odom_set({{{20_in, 46_in,-50_deg}, fwd, 117},
                    {{-13_in, 46_in,-90_deg}, fwd, 127},
                    {{-22_in, 46_in,-90_deg}, fwd, 100},
                   },
                    true);
  chassis.pid_wait_quick_chain();

   chassis.pid_turn_set(-135_deg, 100);
  chassis.pid_wait_quick_chain();

   chassis.pid_drive_set(-13_in, 110, true);
  chassis.pid_wait_quick_chain();

   intake.move(10);
            topintake.move(-10);
      

    chassis.pid_drive_set(-9_in, 90, true);
  chassis.pid_wait();

      intake.move(-127);
            topintake.move(127);
              med.set(false);
             small.set(true);   

      pros::delay(700);

      
      intake.move(0);
            topintake.move(0);
              med.set(false);
             small.set(false);  
             //matchload.set(true); 

       chassis.pid_odom_set({{-45_in, 34_in}, fwd, 127});
  chassis.pid_wait_quick_chain();

  intake.move(-127);
            topintake.move(127);

    chassis.pid_turn_set(180_deg, 127);
  chassis.pid_wait_quick_chain();

/*

      intake.move(-127);
            topintake.move(127);

   chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

    chassis.pid_drive_set(8_in, 127, true);
  chassis.pid_wait_quick_chain();

  pros::delay(75);

   chassis.pid_drive_set(-5_in, 127, true);
  chassis.pid_wait_quick_chain();
*/
  
       chassis.pid_odom_set({{-48_in, 50_in}, rev, 127});
  chassis.pid_wait_quick_chain();

   med.set(true);
  small.set(false);  

  
      intake.move(-127);
            topintake.move(127);
             matchload.set(false);

      pros::delay(3000);




    
}

int normal = 110;

void sixThreeLow() {



  
  chassis.odom_xyt_set(19.5_in, 7.5_in, 0_deg);

  intake.move(-127);
            topintake.move(127);



   chassis.pid_odom_set({{{19.5_in, 20_in}, fwd, normal},
                        { {24_in, 42_in}, fwd, 60}},
                        true);
    chassis.pid_wait();
   matchload.set(true);


     chassis.pid_odom_set({{{25_in, 45_in}, fwd, normal},
                        },
                        true);
    chassis.pid_wait();

     pros::delay(200);
   

     chassis.pid_odom_set({{{28_in, 52_in}, rev, normal},
                        },
                        true);
                      chassis.pid_wait(); 

     matchload.set(false);      
         pros::delay(200); 

chassis.pid_turn_set(-45_deg,90);
 chassis.pid_wait();   

  chassis.pid_odom_set({{{18_in, 62_in}, fwd, normal},
                        },
                        true);     
   chassis.pid_wait();                

             intake.move(100);
            topintake.move(-100);

            pros::delay(1000);
 
             intake.move(0);
            topintake.move(0);

      chassis.pid_odom_set({{{50_in, 24_in}, rev, normal},
                        },
                        true);
       chassis.pid_wait();

chassis.pid_turn_set(180_deg,90);
 chassis.pid_wait();

 matchload.set(true);
  chassis.pid_wait();

  pros::delay(200);

 
  chassis.pid_drive_set(10_in, 120, true);
  chassis.pid_wait();

   chassis.pid_drive_set(-1_in, 120, true);
  chassis.pid_wait();

   chassis.pid_drive_set(1_in, 120, true);
  chassis.pid_wait();

   chassis.pid_drive_set(-1_in, 120, true);
  chassis.pid_wait();

   chassis.pid_drive_set(1_in, 120, true);
  chassis.pid_wait();

    chassis.pid_drive_set(-1_in, 120, true);
  chassis.pid_wait();

   chassis.pid_drive_set(1_in, 120, true);
  chassis.pid_wait();

    chassis.pid_drive_set(-1_in, 120, true);
  chassis.pid_wait();

   chassis.pid_drive_set(4_in, 120, true);
  chassis.pid_wait();
  pros::delay(400);


    chassis.pid_odom_set({{{52_in, 24_in}, rev, normal},
                        },
                        true);


   chassis.pid_drive_set(-30_in, 110, true);
  chassis.pid_wait();

       intake.move(-127);
            topintake.move(127);
              med.set(true);
             small.set(false);

             pros::delay(3000);

}

void sixThree() {



  chassis.odom_xyt_set(-19.5_in, 7.5_in, 0_deg);
    chassis.odom_x_flip();
  chassis.odom_theta_flip();

  intake.move(-127);
            topintake.move(127);



   chassis.pid_odom_set({{{19.5_in, 20_in}, fwd, normal},
                        { {23_in, 40_in}, fwd, 60}},
                        true);
    chassis.pid_wait();
   matchload.set(true);


     chassis.pid_odom_set({{{25_in, 45_in}, fwd, normal},
                        },
                        true);
    chassis.pid_wait();

     pros::delay(200);
   



     matchload.set(false);      
         pros::delay(200); 

chassis.pid_turn_set(135_deg,90);
 chassis.pid_wait();   

  chassis.pid_odom_set({{{15.5_in, 57_in}, rev, normal},
                        },
                        true);     
   chassis.pid_wait();       

    chassis.pid_drive_set(-7_in, 50, true);
  chassis.pid_wait();         

             intake.move(-100);
            topintake.move(100);

                   med.set(false);
             small.set(true);

            pros::delay(1000);


             intake.move(10);
            topintake.move(-10);
            pros::delay(100);
             small.set(false);
             med.set(false);

      chassis.pid_odom_set({{{49_in, 24_in}, fwd, normal},
                        },
                        true);
       chassis.pid_wait();
           intake.move(0);
            topintake.move(0);

chassis.pid_turn_set(180_deg,90);
 chassis.pid_wait();

 matchload.set(true);
  chassis.pid_wait();

  pros::delay(200);

    intake.move(-127);
    topintake.move(127);
       

 
  chassis.pid_drive_set(9_in, 120, true);
  chassis.pid_wait();





   chassis.pid_drive_set(-30_in, 110, true);
  chassis.pid_wait();

       intake.move(-127);
            topintake.move(127);
              med.set(true);
             small.set(false);

             pros::delay(3000);

}


void skillsWait() {
  chassis.odom_xyt_set(-43_in, 48_in, 180_deg);
     chassis.pid_odom_set({{{-44_in, 35_in,170_deg}, fwd, 50},
                        {{-20_in, 12_in,100_deg}, fwd, 50}},
                         true);
chassis.pid_wait();


  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 50, 10);
  chassis.pid_wait();

    chassis.pid_drive_set(-10_in, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(70_in, 127, true);
  chassis.pid_wait();
}

void hi() {


  
  chassis.pid_drive_set(-6_in, 90, true);
  chassis.pid_wait();


}


void nineBlock() {
  // ANISH do it here
}