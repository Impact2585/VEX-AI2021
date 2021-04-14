#include "vex.h"
#include "ballStorage.h"
#include "robotMap.h"

using namespace vex;

ballStorage::ballStorage(){}

void ballStorage::run_intake(double speed){
  Intake.spin(directionType::fwd, speed, percentUnits::pct);
}
void ballStorage::run_index(double speed){
  Indexer.spin(directionType::fwd, speed, percentUnits::pct);
}
void ballStorage::run_shooter(double speed){
  Shooter.spin(directionType::fwd, speed, percentUnits::pct);
}

void ballStorage::intake(double speed){
  run_intake(speed);
  run_index(speed);
}

void ballStorage::shoot(double speed){
  run_index(speed);
  run_shooter(speed);
}
