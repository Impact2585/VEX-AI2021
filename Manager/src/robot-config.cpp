#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller;

motor left1 = motor(PORT1, ratio18_1, false);
motor left2 = motor(PORT2, ratio18_1, false);
motor right1 = motor(PORT11, ratio18_1, true); 
motor right2 = motor(PORT12, ratio18_1, true);
motor_group left_drive = motor_group(left1, left2);
motor_group right_drive = motor_group(right1, right2);
motor intake1 = motor(PORT14, ratio18_1, false); 
motor intake2 = motor(PORT15, ratio18_1, true);
motor_group Intake = motor_group(intake1, intake2);
motor Indexer = motor(PORT18, ratio18_1, true);
motor Shooter = motor(PORT16, ratio18_1, true);
drivetrain Drivetrain = drivetrain(left_drive, right_drive);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}