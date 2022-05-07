#include "motor_CIM.h"
#include "XDrive.h"
#include "vex.h"
using namespace vex;

extern brain Brain;

// VEXcode devices

extern controller Remote;
extern vision::signature BLACKLINE;
extern vision FrontLeftCamera;
extern vision FrontRightCamera;
extern vision BackLeftCamera;
extern vision BackRightCamera;
extern inertial LeftGyro;
extern inertial RightGyro;

extern encoder LeftFlyWheelEncoder;
// extern encoder RightFlyWheelEncoder;

extern motor_accel_cim intakes, flywheel;
    
extern Chassis chassis;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);