#include "vex.h"
#include "ballStorage.h"
#include "robotMap.h"

using namespace vex;

ballStorage::ballStorage(){}

void ballStorage::intake(double speed){
  Intake.spin(directionType::fwd, speed, percentUnits::pct);
}

void ballStorage::shoot(double speed){
  Shooter.spin(directionType::fwd, speed, percentUnits::pct);

}
