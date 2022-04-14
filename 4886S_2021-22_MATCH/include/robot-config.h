#ifndef RC_DEFINED
#define RC_DEFINED

#define DEADBAND 100

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>

#include "v5.h"
#include "v5_vcs.h"

#include "stddefs.h"

// Normal Devices
//  6_1 = speed motor, 600 RPM
//  18_1 = normal motor, 200 RPM
//  36_1 = torque motor, 100 RPM
using namespace vex;
brain Brain;
controller Controller;

motor motorDBL (PORT1, ratio6_1, true);
motor motorDBR (PORT2, ratio6_1, false);
motor motorDFL (PORT3, ratio6_1, true);
motor motorDFR (PORT4, ratio6_1, false);
motor motorDTL (PORT5, ratio6_1, true);
motor motorDTR (PORT6, ratio6_1, false);
motor rings (PORT7, ratio6_1, true);
motor lift (PORT8, ratio36_1, true);

pneumatics claw (Brain.ThreeWirePort.A);
pneumatics armL (Brain.ThreeWirePort.C);
pneumatics armR (Brain.ThreeWirePort.B);

inertial Gyro (PORT20);

#endif