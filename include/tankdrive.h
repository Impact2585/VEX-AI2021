#pragma once
#include "vex.h"

using namespace vex;

class tankDrive{
public:
  motor Motor_Left_1;
  motor Motor_Left_2;
  motor Motor_Right_1;
  motor Motor_Right_2;
  double turn_kP;
  double move_kP;
  double BUFFER;

  tankDrive();

  void move_left_side(double);
  void move_right_side(double);

  // Turning is in units of degrees
  void left_turn(double);
  void right_turn(double);

  double turn_speed(double, double);
  double move_speed(double, double);
  bool move(double, double, double, double);
};