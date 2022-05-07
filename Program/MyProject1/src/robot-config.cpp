#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Remote = controller(primary);
motor LeftRearDrive(PORT1, false);
motor LeftRearMDrive(PORT2, true);
motor LeftFrontMDrive(PORT3, false);
motor LeftFrontDrive(PORT4, true);
motor RightRearDrive(PORT7, false);
motor RightRearMDrive(PORT8, true);
motor RightFrontMDrive(PORT9, false);
motor RightFrontDrive(PORT10, true);
motor Shifter(PORT21, true);
motor Intake(PORT6, true);
motor SnatchClaw(PORT19, true);

motor_group LeftDrive(LeftRearDrive, LeftRearMDrive, LeftFrontMDrive,
                      LeftFrontDrive);
motor_group RightDrive(RightRearDrive, RightRearMDrive, RightFrontMDrive,
                       RightFrontDrive);

void vexcodeInit(void) {
  LeftDrive.setStopping(coast);
  RightDrive.setStopping(coast);
}