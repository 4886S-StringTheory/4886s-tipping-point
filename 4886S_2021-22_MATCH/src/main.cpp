#include "robot-config.h"
#include "helpfuncs.h"

competition Competition;

#define RATIO_3_7 (7/3)

#define AUTON_SELECTOR 2;
#define DRIVE_TYPE 1
#define RANGE 0.5
#define SENSITIVITY 1
#define OUT_OF_AIR 24

#define currentPos (driveLeft.position(ROT_REV) + driveRight.position(ROT_REV)) / 2

// Motor Groups
motor_group driveRight(motorDFR, motorDBR, motorDTR);
motor_group driveLeft(motorDFL, motorDBL, motorDTL);
motor_group driveFull(motorDFR, motorDBR, motorDTR, motorDFL, motorDBL, motorDTL);

// Globals
float drive_speed = 1;
bool claw_open = true;
int cswap_count = 0;

// Prototypes
void pre_auton(void);
void autonomous(void);
void usercontrol(void);

void pid_move(float deg, float dist, float speed);
void move_drive(int dist, int speed, bool waitForCompletion = true);
void move_lift(float height, float speed, bool waitForCompletion);

void claw_swap(void);

void dswap_spd(void);

void print_info(void);

// Main will set up the competition functions and callbacks.
int main() {
  pre_auton();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  // Callbacks
  BTN_B.released(dswap_spd);
  BTN_L2.pressed(claw_swap);

  while (true) {
    task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources.
  }      
}

void pre_auton(void) {
  claw.open();
  driveFull.resetPosition();
  rings.resetPosition();
  lift.resetPosition();

  // Calibrate inertial
  CSCREEN.clearScreen();
  CSCREEN.print("Calibrating . . .");
  CSCREEN.newLine();
  Gyro.calibrate();
  while (Gyro.isCalibrating()) wait(20, msec);
  CSCREEN.print("Calibrated!");
  wait(1, sec);

  // Print settings
  print_info();
}

void autonomous(void) {
  int auton = AUTON_SELECTOR;
  switch (auton) {
    // Right
    case 0:
      break;
    // Left
    case 1:
    pid_move(90, 24, 50);
      break;
    // Basic
    case 2:
      claw.close();
      move_drive(43, 100);
      claw.open();
      move_drive(-40, 100);
      break;
    // None
    case 3:
      break;
  }
}

void usercontrol(void) {
  print_info();
  while (1) {
    // Drive
    // Implementation of tank drive
    if (DRIVE_TYPE == 0) {
      driveRight.spin(DIR_FWD, R_STICK_VRT * drive_speed, VEL_PCT);
      driveLeft.spin(DIR_FWD, L_STICK_VRT * drive_speed, VEL_PCT);
      if (drive_speed == 0.5 && R_STICK_VRT == 0 && L_STICK_VRT == 0) driveFull.stop(brakeType::hold);
    }
    // Implementation of one stick arcade drive
    else if (DRIVE_TYPE == 1) {
      driveRight.spin(DIR_FWD, (L_STICK_VRT - SENSITIVITY  * L_STICK_HRZ) * drive_speed, VEL_PCT);
      driveLeft.spin(DIR_FWD, (L_STICK_VRT + SENSITIVITY * L_STICK_HRZ) * drive_speed, VEL_PCT);
      if (drive_speed == 0.5 && L_STICK_HRZ == 0 && L_STICK_VRT == 0) driveFull.stop(brakeType::hold);
    }
    // Implementation of two stick arcade drive
    else {
      driveRight.spin(DIR_FWD, (L_STICK_VRT - SENSITIVITY * 7/4 * R_STICK_HRZ) * drive_speed, VEL_PCT);
      driveLeft.spin(DIR_FWD, (L_STICK_VRT  + SENSITIVITY * 7/4 * R_STICK_HRZ) * drive_speed, VEL_PCT);
      if (drive_speed == 0.5 && R_STICK_HRZ == 0 && L_STICK_VRT == 0) driveFull.stop(brakeType::hold);
    }

    // Lift
    if (BTN_R2.pressing() - BTN_R1.pressing() == 0) lift.stop(brakeType::hold);
    else lift.spin(DIR_FWD, 100 * (BTN_R2.pressing() - BTN_R1.pressing()), VEL_PCT);

    task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

// Autonomous functions
void move_drive(int dist, int speed, bool waitForCompletion) {
  driveLeft.spinFor(DIR_FWD, dist / 13.1 * RATIO_3_7, ROT_REV, speed, VEL_PCT, false);
  driveRight.spinFor(DIR_FWD, dist / 13.1 * RATIO_3_7, ROT_REV, speed, VEL_PCT, waitForCompletion);
}

void pid_move(float deg, float dist, float speed) {
  printf("\nFunction Start\n");
  float revCount = dist / LRG_OMNI_CIRC;   // revs needed to reach number of inches
  float endPos = currentPos + revCount;
  float target = HEADING + deg;
  float error = target - HEADING;
  float runTime = 0;
  int dir = -2 * (currentPos > endPos) + 1;   // -1 for rev, 1 for fwd
  speed = speed / 100;    // convert to percentage
  float kP = 1;   // changes based on robot specs?

  // Start moving
  driveFull.spin(DIR_FWD);
  // Rotation - exits when error is within range
  while (-RANGE > error || error > RANGE) {
    if (runTime == 0) printf("Turning\n");
    // Break after 5 sec
    if (runTime > 5) break;
    driveLeft.setVelocity(kP * error, VEL_PCT);
    driveRight.setVelocity(-kP * error, VEL_PCT);
    runTime += 0.05;
    wait(50, msec);
    error = target - HEADING;
    printf("%lf", HEADING);
  }

  printf("Moving\n");
  // Translation
  while(currentPos > endPos + RANGE || currentPos < endPos - RANGE) {
    driveLeft.setVelocity(kP * error + 100 * speed * dir, VEL_PCT);
    driveRight.setVelocity(-kP * error + 100 * speed * dir, VEL_PCT);
    wait(50, msec);
    error = target - HEADING;
    dir = -2 * (currentPos > endPos) + 1;
  }
  printf("Out");
  driveFull.stop(brakeType::hold);
}

void move_lift(float height, float speed, bool waitForCompletion) {
  // Converts height in inches to degrees
  double degCount = RATIO_1_7 * (asin((height - 7.75) / 11.5) / PI * 180 + 90 - acos(9.25 / 11.5) / PI * 180);
  lift.spinToPosition(degCount, ROT_DEG, speed, VEL_PCT, waitForCompletion);
  lift.stop(brakeType::hold);
}

void claw_swap(void) {
  // Switch positon of claw
  if (claw_open) {
    claw.close();
    cswap_count++;
  }
  else claw.open();
  claw_open = !claw_open;

  // Warn when air may be low
  if (OUT_OF_AIR - cswap_count == 12) Controller.rumble(".");
  if (OUT_OF_AIR - cswap_count == 3) Controller.rumble("_");
}

void dswap_spd(void) {
  if (drive_speed == 1) drive_speed = 0.5;
  else drive_speed = 1;
}

void print_info(void) {
  CSCREEN.clearScreen();
  CSCREEN.setCursor(1, 1);
  CSCREEN.print("Drive Speed: %f", drive_speed);
  CSCREEN.newLine();
  CSCREEN.print("Batt Lvl: %d%%", Brain.Battery.capacity(percentUnits::pct));
  task::sleep(1000);
}