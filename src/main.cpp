#include "vex.h"
#include "tankDrive.h"
#include "indexer.h"
#include "intake.h"
#include "robotMap.h"

using namespace vex;

// A global instance of competition
competition Competition;

// AI Jetson Nano
ai::jetson  jetson_comms;

// Manager robot
message_link linkA(VEX_LINK, "VRC_2585VEGA_A", linkType::manager);

// Worker robot
// message_link linkA(VEX_LINK, "VRC_2585VEGA_A", linkType::worker);

// define your global instances of motors and other devices here
tankDrive tank;
intake intake;
indexer indexer;

float targetX, targetY, targetAZ;
bool targeting;

FILE *fp = fopen("/dev/serial2","wb");

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  return;
}

void play(void) {
  // 1. Drive to the highway
    // a. Turn to target heading


    // b. Drive straight to the highway

    // c. Turn to align with highway

  // 2. Drive on the highway
    // a. Follow the highway

  // 3. Drive to the destination
    // a. Turn to the target heading

    // b. Drive straight to destination

    // c. Turn to target heading
}

void run(void) {
  // User control code here, inside the loop
  static MAP_RECORD local_map;
  thread t1(dashboardTask);
  thread t2(play);

  while (1) {

    jetson_comms.get_data( &local_map );
    fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az  );

    for(MAP_OBJECTS each: local_map.mapobj){
      fprintf(fp, "%ld %ld %.2f %.2f %.2f", each.age, each.classID, each.p[0], each.p[1], each.p[2]);
    }

    // request new data        
    jetson_comms.request_map();
    this_thread::sleep_for(20);
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
