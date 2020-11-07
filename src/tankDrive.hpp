#pragma once
#include "vex.h"

using namespace vex;

class tankDrive{
public:
  motor Motor_Left_1;
  motor Motor_Left_2;
  motor Motor_Right_1;
  motor Motor_Right_2;
  tankDrive();
  void move_left_side(int);
  void move_right_side(int);
  void turn_clockwise(int);
  void turn_counterclockwise(int);
};
