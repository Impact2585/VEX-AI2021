/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Shaowen                                          */
/*    Created:      Fri Nov 06 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "tankDrive.h"

using namespace vex;
tankDrive tank();
controller master(primary);
auto Left = master.Axis4;
auto Right = master.Axis2;
auto buttonL1 = master.ButtonL1;
auto buttonR1 = master.ButtonR1;
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while(true) {
    double LeftSpeed = Left.value();
    double RightSpeed = Right.value();
    tank().move_left_side(LeftSpeed);
    tank().move_right_side(RightSpeed);
    if(buttonL1.pressing() == true) {
      tank().turn_clockwise(200);
    }
    else if(buttonR1.pressing()) {
      tank().turn_counterclockwise(200);
    }
    Brain.sleep(50);
  }
}
