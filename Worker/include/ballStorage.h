#pragma once
#include "vex.h"

using namespace vex;

class ballStorage{
public:
  ballStorage();
  void run_intake(double);
  void run_index(double);
  void run_shooter(double);

  void intake(double);
  void shoot(double);
};