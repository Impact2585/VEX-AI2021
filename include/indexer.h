#pragma once
#include "vex.h"

using namespace vex;

class indexer{
public:
  motor indexer_motor;
  indexer();
  void index(double);
};