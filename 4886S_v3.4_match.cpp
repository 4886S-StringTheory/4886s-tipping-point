#include "robot-config.h"
 
competition Competition;

#define AUTON_SELECTOR 0;
#define RANGE 0.5
#define currentPos (driveLeft.position(rotationUnits::rev) + driveRight.position(rotationUnits::rev)) / 2

const double PI = acos(0) * 2;

// Motor Groups
motor_group driveRight(motorDFR, motorDBR);
motor_group driveLeft(motorDFL, motorDBL);
motor_group driveFull(motorDFR, motorDFL, motorDBR, motorDBL);
motor_group lift(liftR, liftL);

bool preAutonCompleted = false;

// Globals for settings
float dspeed = 1;
float lspeed = 1;
bool tank = false;
bool clawType = false;
int ddir = 1;

void pre_auton(void);
void autonomous(void);
void usercontrol(void);

// Prototypes for autonomous
void pid_move(float deg, float dist, float speed);
void auto_lift(float height, float speed, bool waitForCompletion);
void auto_claw(bool closed, bool waitForCompletion);

// Prototypes for settings
void dswap_type(void);
void dswap_dir(void);
void dswap_spd(void);
void lswap_spd(void);
void swap_claw(void);
void close_claw(void);
void open_claw(void);

void set_claw(void);

void print_info(void);

// Main will set up the competition functions and callbacks.
int main() {
  pre_auton();
  while (!preAutonCompleted) wait(20, msec);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  // Callbacks for settings
  control.ButtonY.released(dswap_type);
  control.ButtonA.released(dswap_dir);
  control.ButtonX.released(dswap_spd);
  control.ButtonUp.released(lswap_spd);
  control.ButtonDown.pressed(swap_claw);
  if (clawType) control.ButtonLeft.pressed(close_claw);
  else control.ButtonLeft.pressed(open_claw);

  bmpr.pressed(set_claw);

  while (true) {
    task::sleep(100); // Sleep the task for a short amount of time to prevent wasted resources.
  }      
}

void pre_auton(void) {
  driveFull.resetPosition();
  lift.resetPosition();
  claw.resetPosition();
  driveFull.setStopping(brakeType::hold);
  lift.setStopping(brakeType::hold);
  claw.setStopping(brakeType::hold);

  // Calibrate inertial
  control.Screen.clearScreen();
  control.Screen.print("Calibrating . . .");
  control.Screen.newLine();
  inrtl.calibrate();
  while (inrtl.isCalibrating()) wait(20, msec);
  control.Screen.print("Calibrated!");
  wait(1, sec);
  print_info();
}

void autonomous(void) {
  int auton = AUTON_SELECTOR;
  claw.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(500, msec);
  auto_claw(false, true);

  switch (auton) {
    // Right
    case 0:
      // Drives to yC, pulls it back
      pid_move(0, 48, 60);
      auto_claw(true, true);
      pid_move(0, -32, 60);
      break;
    // Left
    case 1:
      // Drives to yA, pulls it back
      pid_move(0, 52, 60);
      auto_claw(true, true);
      pid_move(0, -38, 50);
      break;
    // None
    case 2:
      break;
  }
}

void usercontrol(void) {
  while (1) {
    // Drive
    // Implementation of tank drive
    if (tank) {
      driveRight.spin(directionType::fwd, control.Axis2.value() * ddir * dspeed, velocityUnits::pct);
      driveLeft.spin(directionType::fwd, control.Axis3.value() * ddir * dspeed, velocityUnits::pct);
    }
    // Implementation of arcade drive
    else {
      driveRight.spin(directionType::fwd, (control.Axis3.value() * ddir - 2 * control.Axis1.value()) * dspeed, pct);
      driveLeft.spin(directionType::fwd, (control.Axis3.value() * ddir + 2 * control.Axis1.value()) * dspeed, pct);
    }

    // Lift
    lift.spin(directionType::fwd, (control.ButtonR1.pressing() - control.ButtonR2.pressing()) * 100 * lspeed, velocityUnits::pct);
    if (control.ButtonR1.pressing() - control.ButtonR2.pressing() == 0) lift.stop();

    // Claw
    claw.spin(directionType::fwd, (control.ButtonL2.pressing() - control.ButtonL1.pressing()) * 100, velocityUnits::pct);
    if (control.ButtonL2.pressing() - control.ButtonL1.pressing()) claw.stop();

    // End of usercontrol
    task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

// Autonomous functions
void pid_move(float deg, float dist, float speed) {
  printf("\nFunction Start\n");
  float revCount = dist / 12.95906;   // revs needed to reach number of inches
  float endPos = currentPos + revCount;
  float target = inrtl.rotation() + deg;
  float error = target - inrtl.rotation();
  float runTime = 0;
  int dir = -2 * (currentPos > endPos) + 1;   // -1 for rev, 1 for fwd
  speed = speed / 100;    // convert to percentage
  float kP = 1;   // changes based on robot specs?

  // Start moving
  driveFull.spin(directionType::fwd);
  // Rotation - exits when error is within range
  while (-RANGE > error || error > RANGE) {
    if (runTime == 0) printf("Turning\n");
    // Break after 5 sec
    if (runTime > 5) break;
    driveLeft.setVelocity(kP * error, velocityUnits::pct);
    driveRight.setVelocity(-kP * error, velocityUnits::pct);
    runTime += 0.05;
    wait(50, msec);
    error = target - inrtl.rotation();
    printf("%lf", inrtl.rotation());
  }

  printf("Moving\n");
  // Translation
  while(currentPos > endPos + RANGE || currentPos < endPos - RANGE) {
    driveLeft.setVelocity(kP * error + 100 * speed * dir, velocityUnits::pct);
    driveRight.setVelocity(-kP * error + 100 * speed * dir, velocityUnits::pct);
    wait(50, msec);
    error = target - inrtl.rotation();
    dir = -2 * (currentPos > endPos) + 1;
  }
  printf("Out");
  driveFull.stop(brakeType::hold);
}

void move_lift(float height, float speed, bool waitForCompletion) {
  // Converts height in inches to degrees
  double degCount = 5 * (asin((height - 7.75) / 11.5) / PI * 180 + 90 - acos(9.25 / 11.5) / PI * 180);
  lift.spinToPosition(degCount, rotationUnits::deg, speed, velocityUnits::pct, waitForCompletion);
  lift.stop(brakeType::hold);
}

void auto_claw(bool closed, bool waitForCompletion) {
  if (closed) {
    claw.spin(directionType::fwd, 100, velocityUnits::pct);
    while (!bmpr.pressing() && waitForCompletion) wait(20, msec);
  }
  else {
    claw.spinToPosition(-0.9, rotationUnits::rev, 100, velocityUnits::pct, waitForCompletion);
  }
  claw.stop();
}

void open_claw(void) {claw.spinToPosition(0.9, rotationUnits::rev, 100, velocityUnits::pct, false);}
void close_claw(void) {claw.spin(directionType::fwd, 100, velocityUnits::pct);}

// Functions for settings
void dswap_type(void) {tank = !tank;}
void dswap_dir(void) {ddir = ddir * -1;}
void dswap_spd(void) {
  if (dspeed == 1) {
    dspeed = 0.75;
  } else dspeed = 1;
}
void lswap_spd(void) {
  if (lspeed == 1) {
    lspeed = 0.5;
  } else lspeed = 1;
}
void swap_claw(void) {clawType = !clawType;}

void set_claw(void) {
  claw.resetPosition();
  claw.stop();
}

void print_info(void) {
  control.Screen.clearScreen();
  control.Screen.setCursor(1, 1);
  if (tank) {
    control.Screen.print("D_Type: Tank");
  }
  else {
    control.Screen.print("D_Type: Arcade");
  }
  if (ddir == 1) {
    control.Screen.print(", F");
  }
  else {
    control.Screen.print(", R");
  }
  control.Screen.setCursor(2, 1);
  control.Screen.print("D_Speed: %f", dspeed);
  control.Screen.setCursor(3, 1);
  control.Screen.print("L_speed: %f", lspeed);
}
