#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor DRIVE_RF = motor(PORT1, true);
motor DRIVE_RB = motor(PORT7, true);
motor DRIVE_LF = motor(PORT2, false);
motor DRIVE_LB = motor(PORT10, false);

motor RAMP = motor(PORT19, false);

pwm_out Led1 = pwm_out(Brain.ThreeWirePort.B);

encoder encoderL = encoder(Brain.ThreeWirePort.F);
encoder encoderR = encoder(Brain.ThreeWirePort.C);
encoder encoderB = encoder(Brain.ThreeWirePort.D);

motor INTAKE_R = motor(PORT6);
motor INTAKE_L = motor(PORT8, true);

motor ARM = motor(PORT9);

controller controller1 = controller();

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
