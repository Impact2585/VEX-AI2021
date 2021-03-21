#include "vex.h"
#include "tankDrive.h"
#include "indexer.h"
#include "intake.h"
#include "robotMap.h"
#include <cmath>
#include <vector>
#include <algorithm>

using namespace vex;
using namespace std;

#define DISTANCE_BUFFER 1.0
#define ANGLE_BUFFER 5.0
#define STOP_BEFORE 12.0
#define GOAL_CONST_BEFORE 24.0
#define GOAL_CONST 34.0

// 0 is red team, 1 is blue team. Change and restart before matches
#define TEAM_COLOR 1

// A global instance of competition
competition Competition;

// AI Jetson Nano
ai::jetson  jetson_comms;
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT11, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT11, "robot_32456_1", linkType::worker );
#endif

// define your global instances of motors and other devices here
static tankDrive tank;
static intake intake;
static indexer indexer;

float targetX = 0.0, targetY = 0.0;
int phase = 1;
int drivePhase = 1;
int highwaySeg = -1;
int curGoal = 0;
int32_t loop_time = 66;
static MAP_RECORD local_map;
static vector<MAP_OBJECTS> ballsInGoal;

FILE *fp = fopen("/dev/serial2","wb");

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  return;
}

bool orderByHeight (MAP_OBJECTS i, MAP_OBJECTS j) {
  return i.positionZ < j.positionZ;
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
        return true;
      }
      break;
    }
  }

  return false;
}


void play(bool isolation) {
  tuple<pair<double, double>, double> res = tuple<pair<double, double>, double> {pair<double,double>{0.0, 0.0}, 0.0};

  while(1){
    fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);

    for(MAP_OBJECTS each: local_map.mapobj){
      fprintf(fp, "%ld %ld %.2f %.2f %.2f", each.age, each.classID, each.positionX, each.positionY, each.positionZ);
    }
    
    switch(phase){
      case 1: { // find ball
        int camRange = 60;
        float bestX;
        float bestY;
        float roboX = local_map.pos.x;
        float roboY = local_map.pos.y;
        float bestDist = 100;
        for (int i = 0; i<360/camRange; i++){

          for(MAP_OBJECTS each: local_map.mapobj){
            if(each.classID == TEAM_COLOR){
              if((abs(each.positionX - GOAL_CONST) < DISTANCE_BUFFER || abs(each.positionX + GOAL_CONST) < DISTANCE_BUFFER || abs(each.positionX) < DISTANCE_BUFFER) && (abs(each.positionY - GOAL_CONST) < DISTANCE_BUFFER || abs(each.positionY + GOAL_CONST) < DISTANCE_BUFFER || abs(each.positionY) < DISTANCE_BUFFER)){
                float dist = sqrt(pow((roboX-each.positionX),2) + pow((roboY-each.positionY),2));
                // Find X and Y coordinates that give smallest distance
                if (dist < bestDist){
                  bestX = each.positionX;
                  bestY = each.positionY;
                  bestDist = dist;
                }
              }
            }
          }
          // Rotate 60 degrees to the next reference frame
          float targetAZ = local_map.pos.az + camRange;
          while(!tank.move(0, 0, local_map.pos.az, targetAZ)){}
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
          targetX = -GOAL_CONST_BEFORE; targetY = GOAL_CONST_BEFORE;
        } else if (curGoal == 1){
          targetX = -GOAL_CONST_BEFORE; targetY = 0;
        } else if (curGoal == 2){
          targetX = -GOAL_CONST_BEFORE; targetY = -GOAL_CONST_BEFORE;
        } else if (curGoal == 3){
          targetX = 0; targetY = -GOAL_CONST_BEFORE;
        } else if (curGoal == 4){
          targetX = GOAL_CONST_BEFORE; targetY = -GOAL_CONST_BEFORE;
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
        ballsInGoal.clear();
        for(MAP_OBJECTS each: local_map.mapobj){
          if((abs(each.positionX - curGoal.x) < DISTANCE_BUFFER && abs(each.positionY - curGoal.y) < DISTANCE_BUFFER){
            ballsInGoal.push_back(each);
          }
        }
        sort(ballsInGoal.begin(), ballsInGoal.end(), orderByHeight);

        printf("%d", ballsInGoal.size());

        tank.move_left_side(50);
        tank.move_right_side(50);

        this_thread::sleep_for(1000);

        tank.move_left_side(0);
        tank.move_right_side(0);
        if(ballsInGoal.size() == 3)
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

        if(ballsInGoal.size() == 3){
          if(ballsInGoal[0].classID == TEAM_COLOR){
            phase = 4;
          } else {
            indexer.index(50);
            this_thread::sleep_for(1000);
            indexer.index(0);
            phase = 1;
          }
        } else {
          phase = 1;
        }
        break;
      }
    }

    this_thread::sleep_for(loop_time);
  }
}

// // Demo message sender in message_link
// int sendDemo() {
//   // wait for link
//   while( !link.isLinked() )
//     this_thread::sleep_for(50);
    
//   // check for connection
//   while(1) {
//     link.send("demoMessage");
//     this_thread::sleep_for(500);
//   }
//   return 0;
// }

// // Demo message receiver in message_link
// void receiveDemo( const char *message, const char *linkname, double value ) {
//   printf("%s: was received on '%s' link\n", message, linkname );
// }

void auto_Isolation(void) {
  play(true);
}

void auto_Interaction(void) {
  play(false);
}

bool firstAutoFlag = true;

void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  thread t1(dashboardTask);
  Competition.autonomous(autonomousMain);
  // linkA.received("demoMessage", receiveDemo);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  // print through the controller to the terminal (vexos 1.0.12 is needed)
    // As USB is tied up with Jetson communications we cannot use
    // printf for debug.  If the controller is connected
    // then this can be used as a direct connection to USB on the controller
    // when using VEXcode.
    //
    //FILE *fp = fopen("/dev/serial2","wb");

    while(1) {
        // get last map data
        jetson_comms.get_data( &local_map );

        // set our location to be sent to partner robot
        link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az );

        fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az  );

        // request new data    
        // NOTE: This request should only happen in a single task.    
        jetson_comms.request_map();

        // Allow other tasks to run
        this_thread::sleep_for(loop_time);
    }
}
