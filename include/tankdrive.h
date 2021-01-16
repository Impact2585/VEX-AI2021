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
  void move_left_side(double);
  void move_right_side(double);
};