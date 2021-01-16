#include "vex.h"

using namespace vex;

motor indexer_motor(3);

void index(double speed) {
  indexer_motor.setVelocity(speed, velocityUnits::rpm);
}

int main() {
  return 0; 
}