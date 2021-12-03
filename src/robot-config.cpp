#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor BL = motor(PORT9, ratio18_1, false);
motor FL = motor(PORT19, ratio18_1, false);
motor FR = motor(PORT16, ratio18_1, true);
motor BR = motor(PORT12, ratio18_1, true);
encoder Left = encoder(Brain.ThreeWirePort.A);
encoder Right = encoder(Brain.ThreeWirePort.C);
pot Potentiometer = pot(Brain.ThreeWirePort.H);
motor ML = motor(PORT6, ratio18_1, true);
motor Conveyor = motor(PORT8, ratio6_1, false);
controller Controller1 = controller(primary);
digital_out Claw = digital_out(Brain.ThreeWirePort.G);
distance Distance = distance(PORT2);
motor Arm = motor(PORT5, ratio36_1, false);
motor MR = motor(PORT15, ratio18_1, false);
digital_out Mogo = digital_out(Brain.ThreeWirePort.E);

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