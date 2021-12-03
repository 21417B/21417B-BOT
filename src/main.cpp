
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// BL                   motor         9
// FL                   motor         19
// FR                   motor         16
// BR                   motor         12
// Left                 encoder       A, B
// Right                encoder       C, D
// Potentiometer        pot           H
// ML                   motor         6
// Conveyor             motor         8
// Controller1          controller
// Claw                 digital_out   G
// Distance             distance      2
// Arm                  motor         5
// MR                   motor         15
// Mogo                 digital_out   E
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

void preauton() { vexcodeInit(); }

// settiings for PD loop. Need to be tuned for acurate movement
double kP = 0.0337;
double kD = 0.0;
double turnkP = 0.09;
double turnkD = 0.007;

// Foward/Backward Constants
double error = 0.0;
double prevError = 0.0;
double derivative = 0.0;

// Turning Constants
double TurnError = 0.0;
double prevTurnError = 0.0;
double TurnDerivative = 0.0;
int maxPower = 12;



void PID(double target) {
  
 // double timesWithinRange = 0.0;
  while (true) {

    // Average Position for calculations
    double LeftPosition = Left.position(deg);
    double RightPosition = Right.position(deg);
    double averagePosition = (LeftPosition + RightPosition) / -2;
    error = target - averagePosition;
    derivative = error - prevError;
    double lateralPower = kP * error + kD * derivative;
    if(lateralPower>maxPower){
      lateralPower=maxPower;
    }

    FR.spin(fwd, lateralPower, voltageUnits::volt);
    BR.spin(fwd, lateralPower, voltageUnits::volt);
    FL.spin(fwd, lateralPower, voltageUnits::volt);
    BL.spin(fwd, lateralPower, voltageUnits::volt);
    ML.spin(fwd, lateralPower, voltageUnits::volt);
    MR.spin(fwd, lateralPower, voltageUnits::volt);

   /** if (std::abs(error) < 10) {
      timesWithinRange++;
    } else {
      timesWithinRange = 0;
    }
     if(timesWithinRange>=10){
       break;
     }**/
     Brain.Screen.print("error:");
     Brain.Screen.print(error);
     Brain.Screen.newLine();
     Brain.Screen.print("avg:");
     Brain.Screen.print(averagePosition);
    error = prevError;
    vex::task::sleep(20);
  }
  return;
}

// PD On/Off swicth
bool running = true;

void drivePID(double turnTarget) {
  double timesWithinRange = 0.0;
  running = true;
  while (running) {
    double LeftPosition = Left.position(deg);
    double RightPosition = Right.position(deg);

    double TurnDifference = RightPosition - LeftPosition;

    TurnError = turnTarget - TurnDifference;
    TurnDerivative = TurnError - prevTurnError;
    // Calculates power the motors recieve
    double TurnMotorPower = (TurnError * turnkP + TurnDerivative * turnkD);
    // Turn Power Output

    FR.spin(fwd, TurnMotorPower, voltageUnits::volt);
    BR.spin(fwd, TurnMotorPower, voltageUnits::volt);
    FL.spin(fwd, TurnMotorPower, voltageUnits::volt);
    BL.spin(fwd, TurnMotorPower, voltageUnits::volt);
    ML.spin(fwd, TurnMotorPower, voltageUnits::volt);
    MR.spin(fwd, TurnMotorPower, voltageUnits::volt);

    // Allows us to gauge how close the Error & TurnError are to 0
    Brain.Screen.print("error:");
    Brain.Screen.print(error);
    Brain.Screen.newLine();
    Brain.Screen.print("TurnError");
    Brain.Screen.print(TurnError);
    // Breaks "while loop" so the program can continue onto next commands
    if (std::abs(TurnError) < 30) {
      timesWithinRange++;
    } else {
      timesWithinRange = 0;
    }
    running = timesWithinRange < 10;
    vex::task::sleep(10);
  }
  return;
}

// Reset Function For Encoders
int reset() {
  Right.resetRotation();
  Left.resetRotation();
  return 0;
}

int ArmStart() {
  Claw = true;
  Claw = false;
  return 0;
}

int MogoClose() {
  Mogo = true;
  return 0;
}

int ConveyorSpin() {
  Conveyor.setVelocity(100, pct);
  Conveyor.spinFor(fwd, 2, rev);

  return 0;
}

// Autonomous
void autonomous(void) {
  // On/Off switch for the PD loop, stops it from running in UserControl
  

  PID(1000);

  // Resets Encoders so a new value can be put in

  // drivePID(2000,-0);

  vex::task::sleep(1000);

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  running = false;
  while (1) {

    // Prevents Robot From Coasting, generally makes movement more precise
    FR.setBrake(hold);
    BR.setBrake(hold);
    FL.setBrake(hold);
    BL.setBrake(hold);

    // Drive Controlls, Split Arcade Drive (Better for Holonomic Drives)
    FR.spin(directionType::fwd,
            Controller1.Axis3.position() - Controller1.Axis1.position(),
            percentUnits::pct);
    BR.spin(directionType::fwd,
            Controller1.Axis3.position() - Controller1.Axis1.position(),
            percentUnits::pct);
    FL.spin(directionType::fwd,
            Controller1.Axis3.position() + Controller1.Axis1.position(), pct);
    BL.spin(directionType::fwd,
            Controller1.Axis3.position() + Controller1.Axis1.position(),
            percentUnits::pct);
    MR.spin(directionType::fwd,
            Controller1.Axis3.position() - Controller1.Axis1.position(),
            percentUnits::pct);
    ML.spin(directionType::fwd,
            Controller1.Axis3.position() + Controller1.Axis1.position(),
            percentUnits::pct);
    wait(20, msec);

    // Arm Controls
    Arm.setVelocity(100, pct);
    if (Controller1.ButtonL2.pressing()) {
      Arm.spin(directionType::rev);
    } else if (Controller1.ButtonL1.pressing()) {
      Arm.spin(directionType::fwd);
    } else if (!Controller1.ButtonL1.pressing() ||
               (!Controller1.ButtonL2.pressing())) {
      (Arm.stop)(brakeType::hold);
    }

    // Claw controls
    if (Controller1.ButtonR1.pressing()) {
      Claw = false;
    } else if (Controller1.ButtonR2.pressing()) {
      Claw = true;
    }

    // Conveyor Controls
    Conveyor.setVelocity(100, pct);
    if (Potentiometer.angle(pct) <= 83) {
      Conveyor.spin(fwd);
    } else if (Potentiometer.angle(pct) >= 84) {
      Conveyor.stop(hold);
    } else if (Controller1.ButtonR2.pressing()) {
      Conveyor.spin(reverse);
    }

    // Mobile Goal Lift Controls
    if (Controller1.ButtonRight.pressing()) {
      Mogo = false;
    } else if (Controller1.ButtonDown.pressing()) {
      Mogo = true;
    }

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  preauton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}