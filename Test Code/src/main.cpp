#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftFront            motor         3               
// LeftMiddle           motor         2               
// LeftBack             motor         1               
// RightFront           motor         13              
// RightMiddle          motor         12              
// RightBack            motor         11              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include <string>

using namespace vex;

competition Competition;

void pre_auton(void) {
  vexcodeInit();

  Brain.Screen.clearScreen();
}

void autonomous(void) {

}

void usercontrol(void) {
 
  while (true) {
    if (abs(Controller1.Axis3.value()) > 10){
      LeftFront.spin(forward, Controller1.Axis3.value(), percent);
      LeftMiddle.spin(forward, Controller1.Axis3.value(), percent);
      LeftBack.spin(forward, Controller1.Axis3.value(), percent);
    }
    if (abs(Controller1.Axis2.value()) > 10){
      RightFront.spin(forward, Controller1.Axis2.value(), percent);
      RightMiddle.spin(forward, Controller1.Axis2.value(), percent);
      RightBack.spin(forward, Controller1.Axis2.value(), percent);
    }
    if (abs(Controller1.Axis3.value()) <= 10){
      LeftFront.spin(forward, 0, percent);
      LeftMiddle.spin(forward, 0, percent);
      LeftBack.spin(forward, 0, percent);
    }
    if (abs(Controller1.Axis2.value()) <= 10){
      RightFront.spin(forward, 0, percent);
      RightMiddle.spin(forward, 0, percent);
      RightBack.spin(forward, 0, percent);
    }

    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
