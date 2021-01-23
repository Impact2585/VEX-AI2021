#pragma once
#include "vex.h"

using namespace vex;

class intake{
public:
  motor intake_motor;
  intake();
  void run_intake(double);
};