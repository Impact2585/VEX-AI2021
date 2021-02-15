#include "vex.h"
#include "indexer.h"
#include "robotMap.h"

using namespace vex;

indexer::indexer() : indexer_motor(INDEXER_MOTOR) {}

void indexer::index(double speed) {
  indexer_motor.setVelocity(speed, velocityUnits::pct);
}
