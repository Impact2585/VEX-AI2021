#include "vex.h"

using namespace vex;

motor indexer(1);

//random subroutines
void index(double speed)
{
  indexer.setVelocity(speed, velocityUnits::rpm);
}

int main()
{
  return 0;
}