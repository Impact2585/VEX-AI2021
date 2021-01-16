#include "vex.h"
#include "robotMap.h"

using namespace vex;

motor Motor_Left_1(LEFT_MOTOR1);
motor Motor_Left_2(LEFT_MOTOR2);
motor Motor_Right_1(RIGHT_MOTOR1);
motor Motor_Right_2(RIGHT_MOTOR2);

void move_left_side(double speed) {
  Motor_Left_1.setVelocity(speed, velocityUnits::rpm);
  Motor_Left_2.setVelocity(speed, velocityUnits::rpm);
}

void move_right_side(double speed) {
  Motor_Right_1.setVelocity(speed, velocityUnits::rpm);
  Motor_Right_2.setVelocity(speed, velocityUnits::rpm);
}