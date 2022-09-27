/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Right1               motor         8               
// Right2               motor         9               
// Left1                motor         1               
// Left2                motor         2               
// Right3               motor         10              
// Left3                motor         3               
// Catapult             motor         11              
// SlipGearSensor       bumper        A               
// USsensor             distance      21              
// Intake               motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
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

void launchCatapult(void) {
  Catapult.spin(forward, 100, percent);
}

void stopCatapult(void) {
  Catapult.stop();
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.

    // Movement
    double forwardValue = Controller1.Axis3.value();
    double turningValue = Controller1.Axis1.value();

    Right1.spin(forward, forwardValue - turningValue, percent);
    Right2.spin(forward, forwardValue - turningValue, percent);
    Right3.spin(forward, forwardValue - turningValue, percent);
    Left1.spin(forward, forwardValue + turningValue, percent);
    Left2.spin(forward, forwardValue + turningValue, percent);
    Left3.spin(forward, forwardValue + turningValue, percent);

    // Catapult with US sensor
  
    if (Controller1.ButtonR1.pressing()) {
       Catapult.spin(forward, 100, percent);
    }
    else if (Controller1.ButtonR2.pressing()) {
       Catapult.spin(reverse, 100, percent);
    }
    else {Catapult.stop(hold);}
    //double distance = USsensor.objectDistance(inches);
    //if (distance <= 1)
    //  stopCatapult();
    
    // Intake
    if (Controller1.ButtonL1.pressing()) {
      Intake.spin(reverse, 100, percent);
    }
    else if (Controller1.ButtonL2.pressing()) {
      Intake.spin(forward, 100, percent);
    }
    else {Intake.stop();}
    
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
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
