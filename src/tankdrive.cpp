#include "vex.h"

using namespace vex;

motor Motor_Left_1(1);
motor Motor_Left_2(2);
motor Motor_Right_1(9);
motor Motor_Right_2(10);

void move_left_side(double speed) {
  Motor_Left_1.setVelocity(speed, velocityUnits::rpm);
  Motor_Left_2.setVelocity(speed, velocityUnits::rpm);
}

void move_right_side(double speed) {
  Motor_Right_1.setVelocity(speed, velocityUnits::rpm);
  Motor_Right_2.setVelocity(speed, velocityUnits::rpm);
}

int main() {
  return 0; 
}