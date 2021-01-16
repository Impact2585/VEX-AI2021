#include "vex.h"

using namespace vex;

motor intake_motor(4);

void run_intake(double speed) {
  intake_motor.setVelocity(speed, velocityUnits::rpm);
}

int main() {
  return 0; 
}