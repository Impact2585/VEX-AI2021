#include "vex.h"

using namespace vex;

motor Indexer_Motor(5); // random motor


//random subroutines 
void runForwards(double speed) {
  Indexer_Motor.setVelocity(speed, velocityUnits::rpm);
}

void runBackwards(double speed) {
  Indexer_Motor.setVelocity(-speed, velocityUnits::rpm);
}

int main() {
  return 0; 
}