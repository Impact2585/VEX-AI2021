#pragma once
#include "vex.h"

using namespace vex;
using namespace std;

class tankDrive{
public:
  motor Motor_Left_1;
  motor Motor_Left_2;
  motor Motor_Right_1;
  motor Motor_Right_2;
  double turn_kP;
  double move_kP;

  tankDrive();

  void move_left_side(double);
  void move_right_side(double);

  // Turning is in units of degrees
  void left_turn(double);
  void right_turn(double);

  double turn_speed(double, double);
  double move_speed(double, double);
  bool move(double, double, double, double);

  tuple<pair<double, double>, double> closestJoinHighway(double, double);
  tuple<pair<double, double>, double> closestLeaveHighway(double, double);

  double angleBetween(double, double, double, double);

};