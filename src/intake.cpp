#include "vex.h"
#include "intake.h"
#include "robotMap.h"

using namespace vex;

intake::intake() : intake_motor(INTAKE_MOTOR){}

void intake::run_intake(double speed) {
  intake_motor.setVelocity(speed, velocityUnits::pct);
}
