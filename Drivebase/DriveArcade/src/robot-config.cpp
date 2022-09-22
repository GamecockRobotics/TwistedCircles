#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Right1 = motor(PORT1, ratio6_1, false);
motor Right2 = motor(PORT2, ratio6_1, true);
motor Left1 = motor(PORT4, ratio6_1, false);
motor Left2 = motor(PORT5, ratio18_1, false);
motor Right3 = motor(PORT3, ratio6_1, false);
motor Left3 = motor(PORT6, ratio6_1, false);
motor Catapult = motor(PORT7, ratio6_1, false);
bumper SlipGearSensor = bumper(Brain.ThreeWirePort.A);
distance USsensor = distance(PORT8);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}