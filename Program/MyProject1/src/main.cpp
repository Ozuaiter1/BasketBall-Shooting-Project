/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\omard                                            */
/*    Created:      Thu Apr 21 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;





int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while(true)
  {
      LeftDrive.spin(fwd,Remote.Axis3.position() - Remote.Axis1.position(),pct);
      RightDrive.spin(fwd,Remote.Axis3.position() + Remote.Axis1.position(),pct);
      Intake.spin(fwd,100*(Remote.ButtonR1.pressing() - Remote.ButtonR2.pressing()),pct);
      SnatchClaw.spin(fwd,100*(Remote.ButtonL1.pressing() - Remote.ButtonL2.pressing()),pct);
      Shifter.spin(fwd,100*(Remote.ButtonUp.pressing() - Remote.ButtonDown.pressing()),pct);
    wait(10,msec);
  }
  
}
