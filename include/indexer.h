#pragma once
#include "vex.h"

using namespace vex;

class indexer
{
public:
  motor indexer1;
  indexer();
  void index(int);
};