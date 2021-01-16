#include "vex.h"
#include "tankdrive.h"
#include "indexer.h"
#include "intake.h"
#include "robotMap.h";

using namespace vex;

// A global instance of competition
competition Competition;

// Manager robot
message_link linkA(VEX_LINK, "VRC_2585VEGA_A", linkType::manager);

// Worker robot
// message_link linkA(VEX_LINK, "VRC_2585VEGA_A", linkType::worker);

// define your global instances of motors and other devices here
tankDrive tank();
intake intake();
indexer indexer();

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  return;
}

void run(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    this_thread::sleep_for(20); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Demo message sender in message_link
int sendDemo() {
  // wait for link
  while( !linkA.isLinked() )
    this_thread::sleep_for(50);
    
  // check for connection
  while(1) {
    linkA.send("demoMessage");
    this_thread::sleep_for(500);
  }
  return 0;
}

// Demo message receiver in message_link
void receiveDemo( const char *message, const char *linkname, double value ) {
  printf("%s: was received on '%s' link\n", message, linkname );
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  // What are these called for AI???
  Competition.autonomous(run);
  Competition.drivercontrol(run);
  linkA.received("demoMessage", receiveDemo);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {

    this_thread::sleep_for(100);
  }
}
