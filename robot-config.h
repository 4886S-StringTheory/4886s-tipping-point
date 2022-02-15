#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

// Normal Devices
//  6_1 = speed motor, 600 RPM
//  18_1 = normal motor, 200 RPM
//  36_1 = torque motor, 100 RPM
using namespace vex;
brain Brain;
controller control;
sonar sonar1 (Brain.ThreeWirePort.B);
bumper bmpr (Brain.ThreeWirePort.A);
timer timeCheck ();

motor motorDFL (PORT1, ratio18_1, false);
motor motorDBL (PORT2, ratio18_1, false);
motor motorDBR (PORT3, ratio18_1, true);
motor motorDFR (PORT4, ratio18_1, true);
motor claw (PORT20, ratio36_1, false);
motor dump (PORT6, ratio18_1, false);
motor liftR (PORT7, ratio36_1, true);
motor liftL (PORT8, ratio36_1, false);