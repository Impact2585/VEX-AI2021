#pragma once
#include "vex.h"

using namespace vex;
using namespace std;

class tankDrive{
public:
  double turn_kP;
  double move_kP;

  tankDrive();

  void move_left_side(double);
  void move_right_side(double);
  void drive(double, double);

  void left_turn(double);
  void right_turn(double);

  // Turning is in units of degrees
  double turn_speed(double, double);
  double move_speed(double, double);
  bool move(double, double, double, double, double);
  bool move(double, double, double, double);

  void drive(double);
  void rotate(double);

  tuple<pair<double, double>, double> closestJoinHighway(double, double);
  tuple<pair<double, double>, double> closestLeaveHighway(double, double);

  double angleBetween(double, double, double, double);
};