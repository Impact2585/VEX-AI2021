#include "vex.h"
#include "robotMap.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

motor left1 = motor(LEFT_MOTOR1, ratio18_1, false);
motor left2 = motor(LEFT_MOTOR2, ratio18_1, false);
motor right1 = motor(RIGHT_MOTOR1, ratio18_1, false); 
motor right2 = motor(RIGHT_MOTOR2, ratio18_1, false);
motor_group left_drive = motor_group(left1, left2);
motor_group right_drive = motor_group(right1, right2);
motor intake1 = motor(INTAKE_MOTOR, ratio18_1, false); 
motor intake2 = motor(INTAKE_MOTOR2, ratio18_1, false);
motor_group Intake = motor_group(intake1, intake2);
motor Indexer = motor(INDEXER_MOTOR, ratio18_1, false);
motor Shooter = motor(SHOOTER_MOTOR, ratio18_1, false);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}