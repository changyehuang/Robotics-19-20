using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor DRIVE_RF, DRIVE_RB, DRIVE_LF, DRIVE_LB, ARM, RAMP, INTAKE_L, INTAKE_R;

extern pwm_out Led1;
extern encoder encoderL, encoderR, encoderB;

extern controller controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
