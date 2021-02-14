#include "vex.h"
#include "robotMap.h"

using namespace vex;

motor Motor_Left_1(LEFT_MOTOR1);
motor Motor_Left_2(LEFT_MOTOR2);
motor Motor_Right_1(RIGHT_MOTOR1);
motor Motor_Right_2(RIGHT_MOTOR2);

void move_left_side(double speed) {
  Motor_Left_1.setVelocity(speed, velocityUnits::pct);
  Motor_Left_2.setVelocity(speed, velocityUnits::pct);
}

void move_right_side(double speed) {
  Motor_Right_1.setVelocity(speed, velocityUnits::pct);
  Motor_Right_2.setVelocity(speed, velocityUnits::pct);
}

void left_turn(double speed){
  Motor_Left_1.setVelocity(-speed, velocityUnits::pct);
  Motor_Left_2.setVelocity(-speed, velocityUnits::pct);

  Motor_Right_1.setVelocity(speed, velocityUnits::pct);
  Motor_Right_2.setVelocity(speed, velocityUnits::pct);
}

void right_turn(double speed){
  Motor_Left_1.setVelocity(speed, velocityUnits::pct);
  Motor_Left_2.setVelocity(speed, velocityUnits::pct);

  Motor_Right_1.setVelocity(-speed, velocityUnits::pct);
  Motor_Right_2.setVelocity(-speed, velocityUnits::pct);
}

void turn_to(double heading, double target){

}

void move_distance(double dist){

}
