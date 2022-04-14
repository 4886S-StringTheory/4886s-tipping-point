#ifndef STDDEF_DEFINED
#define STDDEF_DEFINED

#define PI 3.141592653

// Gear Ratios
#define TRQ_STICK 1
#define NRM_STICK 2
#define SPD_STICK 6
#define TRQ_RPM 100
#define NRM_RPM 200
#define SPD_RPM 600

#define RATIO_3_1 3
#define RATIO_5_1 5
#define RATIO_7_1 7
#define RATIO_1_3 (1/3)
#define RATIO_1_5 (1/5)
#define RATIO_1_7 (1/7)

// Wheels
#define LRG_OMNI_DIAM 4
#define MED_OMNI_DIAM 3.25
#define SML_OMNI_DIAM 2.75
#define LRG_WHEEL_DIAM 5
#define MED_WHEEL_DIAM 4
#define SML_WHEEL_DIAM 2.75
#define TRACT_WHEEL_DIAM 3.25

#define LRG_OMNI_CIRC 13.1
#define MED_OMNI_CIRC 10.2
#define SML_OMNI_CIRC 8.6
#define LRG_WHEEL_CIRC 15.6
#define MED_WHEEL_CIRC 13.1
#define SML_WHEEL_CIRC 8.6
#define TRACT_WHEEL_CIRC 10.2

// Unit Conversions
#define DEG_FULL_ROT 360
#define DEG_HALF_ROT 180
#define DEG_QRTR_ROT 90

#define IN_CM 2.54
#define M_IN 39.37
#define RAD_DEG 57.29578
#define MSEC_SEC 1000

// Devices
// With controller named Controller, brain named Brain and inertial sensor named Gyro
#define L_STICK_VRT Controller.Axis3.value()
#define R_STICK_VRT Controller.Axis2.value()
#define L_STICK_HRZ Controller.Axis4.value()
#define R_STICK_HRZ Controller.Axis1.value()

#define BTN_X Controller.ButtonX
#define BTN_Y Controller.ButtonY
#define BTN_A Controller.ButtonA
#define BTN_B Controller.ButtonB

#define BTN_U Controller.ButtonUp
#define BTN_L Controller.ButtonLeft
#define BTN_R Controller.ButtonRight
#define BTN_D Controller.ButtonDown

#define BTN_L1 Controller.ButtonL1
#define BTN_L2 Controller.ButtonL2
#define BTN_R1 Controller.ButtonR1
#define BTN_R2 Controller.ButtonR2

#define CSCREEN Controller.Screen

#define BSCREEN_WD 480
#define BSCREEN_HI 272

#define BSCREEN Brain.Screen

#define HEADING Gyro.rotation()

// Directions
#define DIR_FWD directionType::fwd
#define DIR_REV directionType::rev

// Units
#define VEL_PCT velocityUnits::pct
#define VEL_RPM velocityUnits::rpm
#define ROT_DEG rotationUnits::deg
#define ROT_REV rotationUnits::rev

#endif