#include "XDrive.h"
#include "motor_CIM.h"
#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Remote(primary);
triport LeftSystemExpander(PORT13);
triport RightSystemExpander(PORT8);
triport RightSensorExpander(PORT20);
triport FlywheelSensorExpander(PORT7);
signature BLACKLINE(1, 387, 795, 590, -5251, -4839, -5044, 6.100, 0);
vision FrontLeftCamera(PORT2, 27, BLACKLINE);
vision FrontRightCamera(PORT9, 27, BLACKLINE);
vision BackLeftCamera(PORT12, 27, BLACKLINE);
vision BackRightCamera(PORT10, 27, BLACKLINE);
inertial LeftGyro(PORT11);
inertial RightGyro(PORT19);

// encoder FrontLeftWheelEncoder(Brain.ThreeWirePort.A);
encoder LeftTracking(Brain.ThreeWirePort.C);
// encoder BackLeftWheelEncoder(Brain.ThreeWirePort.E);
encoder BackTracking(Brain.ThreeWirePort.G);

// encoder FrontRightWheelEncoder(RightSensorExpander.E);
encoder FrontTracking(RightSensorExpander.G);
// encoder BackRightWheelEncoder(RightSensorExpander.C);
encoder RightTracking(RightSensorExpander.A);

encoder LeftFlyWheelEncoder(FlywheelSensorExpander.A);
// encoder RightFlyWheelEncoder(FlywheelSensorExpander.G);

motor_victor FrontLeftWheel(LeftSystemExpander.G);
motor_victor BackLeftWheel(LeftSystemExpander.F);
motor_victor BackIntake(LeftSystemExpander.H);
motor_victor LeftElevator(LeftSystemExpander.D);
motor_victor LeftFlywheelMiddle(LeftSystemExpander.E, true);
motor_victor LeftFlywheelOutside(LeftSystemExpander.C);

motor_victor FrontRightWheel(RightSystemExpander.F, true);
motor_victor BackRightWheel(RightSystemExpander.G, true);
motor_victor FrontIntake(RightSystemExpander.H);
motor_victor RightElevator(RightSystemExpander.C);
motor_victor RightFlywheelMiddle(RightSystemExpander.E);
motor_victor RightFlywheelOutside(RightSystemExpander.D, true);

motor_accel_cim frontLeftWheel({FrontLeftWheel}, 0.125, false);
motor_accel_cim frontRightWheel({FrontRightWheel}, 0.125, true);
motor_accel_cim backLeftWheel({BackLeftWheel}, 0.125, false);
motor_accel_cim backRightWheel({BackRightWheel}, 0.125, true);
motor_accel_cim intakes({FrontIntake, BackIntake, LeftElevator, RightElevator},
                        1, true);
motor_accel_cim flywheel({LeftFlywheelMiddle, LeftFlywheelOutside,
                          RightFlywheelMiddle, RightFlywheelOutside},
                         6, true);
Chassis chassis(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
                LeftGyro, RightGyro, FrontTracking, LeftTracking, RightTracking,
                BackTracking, {{0, 0, 1000000000000000, 5}, {0.03, 10, 1, 15}},
                3.25 / 2.0, 1024, FrontLeftCamera, FrontRightCamera,
                BackLeftCamera, BackRightCamera, BLACKLINE);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {}