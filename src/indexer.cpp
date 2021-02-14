#include "vex.h"
#include "robotMap.h"

using namespace vex;

motor indexer_motor(INDEXER_MOTOR);

void index(double speed) {
  indexer_motor.setVelocity(speed, velocityUnits::pct);
}
