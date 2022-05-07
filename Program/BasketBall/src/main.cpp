//*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\omard                                            */
/*    Created:      Tue Jan 25 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "robot-config.h"
#include "vex.h"

using namespace vex;

int secondaryTask() {
  int count = 0;
  while (true) {
    chassis.calculatePosition();
    chassis.pointControl();
    if (chassis.getMotionControlRunning()) {
      chassis.setSpeed(chassis.getLocalDesiredVelocity(0),
                       chassis.getLocalDesiredVelocity(1),
                       chassis.getLocalDesiredVelocity(2));
    }
    chassis.killSwitch();
    count++;
    if (count > 10) {
      chassis.debug();
      count = 0;
    }
    wait(LOOP_TIME, msec);
  }
}

int main() {
  vexcodeInit();
  chassis.setGyroIsInverted(true);
  chassis.initialize();
  chassis.setMotionControlRunning(false);
  task secondary = task(secondaryTask);
  while (true) {
    chassis.setSpeedDriver(Remote.Axis4.position(), Remote.Axis3.position(),
                           -Remote.Axis1.position());
    intakes.spin(100 *
                 (Remote.ButtonL1.pressing() - Remote.ButtonL2.pressing()));
    flywheel.spin(100 *
                  (Remote.ButtonR1.pressing() - Remote.ButtonR2.pressing()));
    if (Remote.ButtonA.pressing()) {
      chassis.lineUp(false);
      chassis.setMotionControlRunning(false);
    }
    if (Remote.ButtonUp.pressing()) {
      chassis.lookAt(90, true);
      chassis.lookAt(180, true);
      chassis.lookAt(90, true);
      chassis.lookAt(45, true);
      chassis.lookAt(135, true);
      chassis.setMotionControlRunning(false);
    }
    wait(LOOP_TIME, msec);
  }
}
