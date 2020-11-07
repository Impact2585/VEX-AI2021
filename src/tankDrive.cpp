#include "vex.h"

using namespace vex;

motor Motor_Left_1(1);
motor Motor_Left_2(2);
motor Motor_Right_1(9);
motor Motor_Right_2(10);

//random subroutines 
void move_left_side(double speed) {
  Motor_Left_1.setVelocity(speed, velocityUnits::rpm);
  Motor_Left_2.setVelocity(speed, velocityUnits::rpm);
}

void move_right_side(double speed) {
  Motor_Right_1.setVelocity(speed, velocityUnits::rpm);
  Motor_Right_2.setVelocity(speed, velocityUnits::rpm);
}

void turn_clockwise(double speed) {
  move_left_side(speed);
  move_right_side(-speed);
}

void turn_counterclockwise(double speed) {
  move_left_side(-speed);
  move_right_side(speed);
}

int main() {
  return 0; 
}
