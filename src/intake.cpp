#include "vex.h"
#include "robotMap.h"

using namespace vex;

motor intake_motor(INTAKE_MOTOR);

void run_intake(double speed) {
  intake_motor.setVelocity(speed, velocityUnits::pct);
}
