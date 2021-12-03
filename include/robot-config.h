using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor BL;
extern motor FL;
extern motor FR;
extern motor BR;
extern encoder Left;
extern encoder Right;
extern pot Potentiometer;
extern motor ML;
extern motor Conveyor;
extern controller Controller1;
extern digital_out Claw;
extern distance Distance;
extern motor Arm;
extern motor MR;
extern digital_out Mogo;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );