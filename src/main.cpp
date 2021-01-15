// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Indexer.h"

using namespace vex;
Indexer index();
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while(true) {
    double speed = 0; // where do you get the speed from?
    index().runForwards(speed);
    index().runBackwards(speed);
    Brain.sleep(50);
  }
}