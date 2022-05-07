using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Remote;

extern motor_group LeftDrive;
extern motor_group RightDrive;
extern motor Shifter;
extern motor Intake;
extern motor SnatchClaw;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );