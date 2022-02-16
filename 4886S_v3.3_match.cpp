#include "robot-config.h"
#include "helpfunctions.h"
#include "altvex.h"
#include <stdio.h>
#include <math.h>
#include <string>
#include <string.h>
 
competition Competition;

#define AUTON_SELECTOR 0;

// Motor Groups
motor_group driveRight(motorDFR, motorDBR);
motor_group driveLeft(motorDFL, motorDBL);
motor_group driveFull(motorDFR, motorDFL, motorDBR, motorDBL);
motor_group lift(liftR, liftL);

// Globals
float dspeed = 1;
float lspeed = 1;
bool tank = false;
bool clawType = false;
int ddir = 1;
int auton = 0;

void pre_auton(void);
void autonomous(void);
void usercontrol(void);

// Prototypes for movement stuff
void move_drive(float drval, float dlval);
void move_lift(int lval);
void auto_drive(float dist, float speed, bool waitForCompletion);
void rotate_drive(float deg, float speed, bool waitForCompletion);
void auto_lift(float height, float speed, bool waitForCompletion);
void auto_claw(bool closed, bool waitForCompletion);

// Prototypes for settings
void dswap_type(void);
void dswap_dir(void);
void dswap_spd(void);
void lswap_spd(void);
void swap_claw(void);

void auton_down(void);

void close_claw(void);
void open_claw(void);
void set_claw(void);

void print_info(void);

// Main will set up the competition functions and callbacks.
int main() {
  pre_auton();
  print_info();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
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
  // Pre-autonomous, to set up anything that happens before a robot starts
  driveFull.resetPosition();
  lift.resetPosition();
  claw.resetPosition();

  driveFull.setStopping(brakeType::hold);
  lift.setStopping(brakeType::hold);
  claw.setStopping(brakeType::hold);
}

void autonomous(void) {
  auton = AUTON_SELECTOR;
  claw.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(500, msec);
  auto_claw(false, true);

  switch(auton) {
    // Right
    case 0:
      auto_drive(48, 60, true);
      auto_claw(true, true);
      auto_drive(-32, 50, true);
      break;
    // Left
    case 1:
      auto_drive(52, 50, true);
      auto_claw(true, true);
      auto_drive(-38, 50, true);
      break;
    // None
    case 2:
      break;
  }
}

void usercontrol(void) {
  // Acceleration values of drive motors
  float drval = 0;
  float dlval = 0;
  int lval = 0;
  
  // Start user control
  while (true) {
    move_drive(drval, dlval);
    move_lift(lval);
    claw.spin(directionType::fwd, (control.ButtonL2.pressing() - control.ButtonL1.pressing()) * 100, velocityUnits::pct);
    if (control.ButtonL2.pressing() - control.ButtonL1.pressing()) claw.stop();

    // End of usercontrol
    task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

void auto_drive(float dist, float speed, bool waitForCompletion) {
  float revCount = dist / 12.95906;   // 12.95906 is circumference of wheels - also distance traveled in one rotation
  driveRight.spinFor(revCount, rotationUnits::rev, speed, velocityUnits::pct, false);
  driveLeft.spinFor(revCount, rotationUnits::rev, speed, velocityUnits::pct, waitForCompletion);
  driveFull.stop();
}
void rotate_drive(float deg, float speed, bool waitForCompletion) {
  float revCount = deg / 360 * 48.8 / 12.95906;
  driveLeft.spinFor(revCount, rotationUnits::rev, speed, velocityUnits::pct, false);
  driveRight.spinFor(-revCount, rotationUnits::rev, speed, velocityUnits::pct, waitForCompletion);
  driveFull.stop(brakeType::hold);
}
void auto_lift(float height, float speed, bool waitForCompletion) {
  double degCount = 5 * (asin(height - 9.25) + asin(9.25 / 11.5));
  lift.spinToPosition(degCount, rotationUnits::deg, speed, velocityUnits::pct, waitForCompletion);
  lift.stop();
}
void auto_claw(bool closed, bool waitForCompletion) {
  if (closed) {
    claw.spin(directionType::fwd, 100, velocityUnits::pct);
    while (claw.position(rotationUnits::rev) != 0 && waitForCompletion) {
      wait(20, msec);
    }
  }
  else {
    claw.spinToPosition(-0.9, rotationUnits::rev, 100, velocityUnits::pct, waitForCompletion);
  }
  claw.stop();
}

void move_drive(float drval, float dlval) {
  // Implementation of tank drive
  if (tank) {
    dlval = control.Axis3.value() * ddir * dspeed;
    drval = control.Axis2.value() * ddir * dspeed;
  }
  // Implementation of arcade drive
  else {
    drval = (control.Axis3.value() * ddir - 2 * control.Axis1.value()) * dspeed;
    dlval = (control.Axis3.value() * ddir + 2 * control.Axis1.value()) * dspeed;
  }
  // Change motors by given values
  driveRight.spin(directionType::fwd, drval, rpm);
  driveLeft.spin(directionType::fwd, dlval, rpm);
}
void move_lift(int lval) {
  lval = (control.ButtonR1.pressing() - control.ButtonR2.pressing()) * 100 * lspeed;
  if (lval != 0) {
    lift.spin(directionType::fwd, lval, velocityUnits::pct);
  }
  else lift.stop();
}
void open_claw(void) {
  claw.setMaxTorque(100, percentUnits::pct);
  claw.spinToPosition(0.95, rotationUnits::deg, 100, velocityUnits::pct);
  claw.setMaxTorque(10, percentUnits::pct);
  claw.spin(directionType::rev, 100, velocityUnits::pct);
}
void close_claw(void) {
  claw.setMaxTorque(100, percentUnits::pct);
  claw.spinToPosition(0, rotationUnits::deg, 100, velocityUnits::pct);
  claw.setMaxTorque(10, percentUnits::pct);
  claw.spin(directionType::fwd, 100, velocityUnits::pct);
}

void dswap_type(void) {
  tank = !tank;
  control.rumble(".");
  print_info();
}
void dswap_dir(void) {
  ddir = ddir * -1;
  control.rumble("..");
  print_info();
}
void dswap_spd(void) {
  if(dspeed == 1) {
    dspeed = 0.75;
  } else dspeed = 1;
  control.rumble("-");
  print_info();
}
void lswap_spd(void) {
  if(lspeed == 1) {
    lspeed = 0.5;
  } else lspeed = 1;
  control.rumble("-.-");
  print_info();
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
