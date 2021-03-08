#include "vex.h"
#include "tankDrive.h"
#include "indexer.h"
#include "intake.h"
#include "robotMap.h"
#include <cmath>

using namespace vex;
using namespace std;

#define DISTANCE_BUFFER 1.0
#define ANGLE_BUFFER 5.0
#define STOP_BEFORE 12.0 // 

// A global instance of competition
competition Competition;

// AI Jetson Nano
ai::jetson  jetson_comms;

// Manager robot
message_link linkA(VEX_LINK, "VRC_2585VEGA_A", linkType::manager);

// Worker robot
// message_link linkA(VEX_LINK, "VRC_2585VEGA_A", linkType::worker);

// define your global instances of motors and other devices here
static tankDrive tank;
static intake intake;
static indexer indexer;

float targetX = 0.0, targetY = 0.0;
int phase = 1;
int drivePhase = 1;
int highwaySeg = -1;
int curGoal = 0;


FILE *fp = fopen("/dev/serial2","wb");

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  return;
}

bool drive(MAP_RECORD local_map, tuple<pair<double, double>, double> res){
  switch(drivePhase) {
    case 1: { // Turn to target heading
      if(tank.move(0, 0, local_map.pos.az, get<1>(res))){
        drivePhase++;
      }
      break;
    } case 2: { // Drive straight to highway
      int changeX = max(get<0>(res).first, (double)local_map.pos.x) - min(get<0>(res).first, (double)local_map.pos.x);
      int changeY = max(get<0>(res).second, (double)local_map.pos.y) - min(get<0>(res).second, (double)local_map.pos.y);
      if(tank.move(0, sqrt(changeX * changeX + changeY * changeY), local_map.pos.az, get<1>(res))){
        drivePhase++;
      }
      break;
    } case 3: { // Turn to align with highway. Determine which leg of the highway we are on
                // 1 = moving north, 2 = moving east, 3 = moving south, 4 = moving west
      if(abs(local_map.pos.x - -18) < DISTANCE_BUFFER){
        if(tank.move(0, 0, local_map.pos.az, 0)){
          drivePhase++;
          highwaySeg = 1;
        }
      }
      if(abs(local_map.pos.y - 18) < DISTANCE_BUFFER){
        if(tank.move(0, 0, local_map.pos.az, 90)){
          drivePhase++;
          highwaySeg = 2;
        }
      }
      if(abs(local_map.pos.x - 18) < DISTANCE_BUFFER){
        if(tank.move(0, 0, local_map.pos.az, 180)){
          drivePhase++;
          highwaySeg = 3;
        }
      }
      if(abs(local_map.pos.y - -18) < DISTANCE_BUFFER){
        if(tank.move(0, 0, local_map.pos.az, 270)){
          drivePhase++;
          highwaySeg = 4;
        }
      }
      break;
    } case 4: { // Drive until corner
      if(abs(local_map.pos.x - get<0>(res).first) < DISTANCE_BUFFER && abs(local_map.pos.y - get<0>(res).second) < DISTANCE_BUFFER){
        highwaySeg = -1;
        drivePhase += 2;
      }

      switch(highwaySeg){
        case 1:
          if(tank.move(local_map.pos.y, 18, local_map.pos.az, 0)){
            drivePhase++;
          }
          break;
        case 2:
          if(tank.move(local_map.pos.x, 18, local_map.pos.az, 90)){
            drivePhase++;
          }
          break;
        case 3:
          if(tank.move(local_map.pos.y, -18, local_map.pos.az, 180)){
            drivePhase++;
          }
          break;
        case 4:
          if(tank.move(local_map.pos.x, -18, local_map.pos.az, 270)){
            drivePhase++;
          }
          break;
      }
      break;
    } case 5: { // Turn right
      switch(highwaySeg){
        case 1:
          if(tank.move(0, 0, local_map.pos.az, 90)){
            drivePhase--;
            highwaySeg++;
          }
          break;
        case 2:
          if(tank.move(0, 0, local_map.pos.az, 180)){
            drivePhase--;
            highwaySeg++;
          }
          break;
        case 3:
          if(tank.move(0, 0, local_map.pos.az, 270)){
            drivePhase--;
            highwaySeg++;
          }
          break;
        case 4:
          if(tank.move(0, 0, local_map.pos.az, 0)){
            drivePhase--;
            highwaySeg = 1;
          }
          break;
      }
      break;
    } case 6: { // Turn to target heading
      if(tank.move(0, 0, local_map.pos.az, get<1>(res))){
        drivePhase++;
      }
      break; 
    } case 7: { // Drive straight to destination
      int changeX = max(get<0>(res).first, (double)local_map.pos.x) - min(get<0>(res).first, (double)local_map.pos.x);
      int changeY = max(get<0>(res).second, (double)local_map.pos.y) - min(get<0>(res).second, (double)local_map.pos.y);
      if(tank.move(0, sqrt(changeX * changeX + changeY * changeY) - STOP_BEFORE, local_map.pos.az, get<1>(res))){
        drivePhase++;
      }
      break;
    } case 8: {
      return true;
      break;
    }
  }

  return false;
}


void play(void) {
  static MAP_RECORD local_map;
  tuple<pair<double, double>, double> res = tuple<pair<double, double>, double> {pair<double,double>{0.0, 0.0}, 0.0};

  while(1){
    jetson_comms.get_data( &local_map );
    fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);

    for(MAP_OBJECTS each: local_map.mapobj){
      fprintf(fp, "%ld %ld %.2f %.2f %.2f", each.age, each.classID, each.p[0], each.p[1], each.p[2]);
    }

    // request new data        
    jetson_comms.request_map();
    
    switch(phase){
      case 1: { // find ball
        int camRange = 60;
        float bestX;
        float bestY;
        float roboX = local_map.pos.x;
        float roboY = local_map.pos.y;
        float bestDist = 100;
        for (int i = 0; i<360/camRange; i++){
          jetson_comms.get_data( &local_map);

          for(MAP_OBJECTS each: local_map.mapobj){
            float dist = sqrt(pow((roboX-each.p[0]),2) + pow((roboY-each.p[1]),2));
            // Find X and Y coordinates that give smallest distance
            if (dist < bestDist){
              bestX = each.p[0];
              bestY = each.p[1];
              bestDist = dist;
            }
          }
          // Rotate 60 degrees to the next reference frame
          float targetAZ = local_map.pos.az + camRange;
          while(!tank.move(0, 0, local_map.pos.az, targetAZ)){}

          jetson_comms.request_map();
        }
        targetX = bestX;
        targetY = bestY;
        phase++;
        break;
      } case 2: { // drive to ball
        if(drivePhase == 1 || drivePhase == 2)
          res = tank.closestJoinHighway(local_map.pos.x, local_map.pos.y);
        else if (drivePhase == 4 || drivePhase == 6 || drivePhase == 7)
          res = tank.closestLeaveHighway(targetX, targetY); // target location
        
        if(drive(local_map, res)){
          drivePhase = 1; phase++;
        }

        break;
      } case 3: { // intake ball
        intake.run_intake(50);
        tank.move_left_side(50);
        tank.move_right_side(50);

        this_thread::sleep_for(2000);

        intake.run_intake(0);
        tank.move_left_side(-50);
        tank.move_right_side(-50);
        
        this_thread::sleep_for(2000);

        tank.move_left_side(0);
        tank.move_right_side(0);
        phase++;
        break;
      } case 4: { // find goal
        // set targetX, targetY
        // when successful, increment phase
        if(curGoal == 0){
          targetX = -24; targetY = 24;
        } else if (curGoal == 1){
          targetX = -24; targetY = 0;
        } else if (curGoal == 2){
          targetX = -24; targetY = -24;
        } else if (curGoal == 3){
          targetX = 0; targetY = -24;
        } else if (curGoal == 4){
          targetX = 24; targetY = -24;
        }

        phase++;
        curGoal++;
        if(curGoal == 5)
          curGoal = 0;
      } case 5: { // drive to goal

        if(drivePhase == 1 || drivePhase == 2)
          res = tank.closestJoinHighway(local_map.pos.x, local_map.pos.y);
        else if (drivePhase == 4 || drivePhase == 6 || drivePhase == 7)
          res = tank.closestLeaveHighway(targetX, targetY); // target location
        
        if(drive(local_map, res)){
          drivePhase = 1; phase++;
        }

        break;
      } case 6: { // deposit ball
        tank.move_left_side(50);
        tank.move_right_side(50);

        this_thread::sleep_for(1000);

        tank.move_left_side(0);
        tank.move_right_side(0);
        intake.run_intake(50);
        indexer.index(50);

        this_thread::sleep_for(500);

        intake.run_intake(0);
        indexer.index(0);
        tank.move_left_side(-50);
        tank.move_right_side(-50);

        this_thread::sleep_for(1000);

        tank.move_left_side(0);
        tank.move_right_side(0);
        phase = 1;
        break;
      }
    }

    this_thread::sleep_for(20);
  }
}

void run(void) {
  // User control code here, inside the loop
  thread t1(dashboardTask);
  thread t2(play);
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
