#include "vex.h"
#include "tankDrive.h"
#include "ballStorage.h"
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
#define X_MARKS_SPOT 18.0

// 0 is red team, 1 is blue team. Change the program run during competition for rounds. 1 = red team, 2 = blue team
#define TEAM_COLOR 1
// This is the manager robot. THE MANAGER ROBOT IS ALWAYS PLACED TO THE NORTH OF THE WORKER ROBOT.
#define manager_robot true

// A global instance of competition
competition Competition;

// AI Jetson Nano
ai::jetson  jetson_comms;

// #pragma message("building for the manager")
ai::robot_link       link( VEX_LINK, "robot_32456_1", linkType::manager );

// define your global instances of motors and other devices here
static tankDrive tank;
static ballStorage ballStor;

float targetX = 0.0, targetY = 0.0, targetAZ = 315;
int phase = 1;
int drivePhase = 1;
int32_t loop_time = 66;
static MAP_RECORD local_map;

tuple<float, float> redIsolation [] = {tuple<float, float>(-1,1)};
tuple<float, float> blueIsolation [] = {tuple<float, float>(1,1)};
tuple<float, float> interaction [] = {tuple<float, float>(-1,1), tuple<float, float>(1, 1)};

FILE *fp = fopen("/dev/serial2","wb");

bool orderByHeight (MAP_OBJECTS i, MAP_OBJECTS j) {
  return i.positionZ < j.positionZ;
}

bool drive(MAP_RECORD local_map, double tAZ, double tX, double tY){
  switch(drivePhase) {
    case 1: { // Turn to target heading
      if(tank.move(0, 0, local_map.pos.az, tAZ)){
        drivePhase++;
      }
      break;
    } case 2: { // Drive straight to target
      int changeX = max(tX, (double)local_map.pos.x) - min(tX, (double)local_map.pos.x);
      int changeY = max(tY, (double)local_map.pos.y) - min(tY, (double)local_map.pos.y);
      if(tank.move(0, sqrt(changeX * changeX + changeY * changeY), local_map.pos.az, tAZ)){
        drivePhase++;
      }
      break;
    } case 3: { // Turn to final target heading
      if(tank.move(0, 0, local_map.pos.az, tAZ)){
        return true;
      }
      break; 
    }
  }

  return false;
}


void play(bool isolation) {
  while(1){
    fprintf(fp, "ROBOT LOCATION: " );
    fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);
    fprintf(fp, "MAP OBJECTS: \n");
    for(MAP_OBJECTS each: local_map.mapobj){
      fprintf(fp, "%ld %ld %.2f %.2f %.2f", each.age, each.classID, each.positionX, each.positionY, each.positionZ);
    }
    fprintf(fp, "\n");
    
    switch(phase){
      case 1: { // find ball
        int camRange = 60;
        float bestX;
        float bestY;
        float roboX = local_map.pos.x;
        float roboY = local_map.pos.y;
        float bestDist = 100;
        for (int i = 0; i<360/camRange; i+=camRange){

          for(MAP_OBJECTS each: local_map.mapobj){
            if(each.classID == TEAM_COLOR){
              if ((isolation && (
                   (TEAM_COLOR == 0 && manager_robot && each.positionX < 0 && each.positionY > 0)
                || (TEAM_COLOR == 1 && manager_robot && each.positionX > 0 && each.positionY > 0) 
                || (TEAM_COLOR == 0 && !manager_robot && each.positionX < 0 && each.positionY < 0)
                || (TEAM_COLOR == 1 && !manager_robot && each.positionX > 0 && each.positionY < 0)))
              || (!isolation && (
                   (TEAM_COLOR == 0 && manager_robot && each.positionY > 0) 
                || (TEAM_COLOR == 1 && manager_robot && each.positionY > 0) 
                || (TEAM_COLOR == 0 && !manager_robot && each.positionY < 0) 
                || (TEAM_COLOR == 1 && !manager_robot && each.positionY < 0)))){
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
          }
          // Rotate 60 degrees to the next reference frame
          float ang = local_map.pos.az + camRange;
          while(!tank.move(0, 0, local_map.pos.az, ang)){
            this_thread::sleep_for(loop_time);
          }
        }
        targetX = bestX;
        targetY = bestY;
        phase++;
        break;
      } case 2: { // drive to ball
        if(drive(local_map, targetX, targetY, tank.angleBetween(local_map.pos.x, local_map.pos.y, targetX, targetY))){
          drivePhase = 1; phase++;
        }

        break;
      } case 3: { // intake ball
        ballStor.intake(50);
        tank.drive(50, 0);

        this_thread::sleep_for(2000);

        ballStor.intake(0);
        tank.drive(-50, 0);
        
        this_thread::sleep_for(2000);

        tank.drive(0, 0);
        phase++;
        break;
      } case 4: { // find goal
        // set targetX, targetY
        // when successful, increment phase
        if (TEAM_COLOR == 0) {
          targetX = get<0>(redIsolation[0])*X_MARKS_SPOT;
          targetY = get<1>(redIsolation[0])*X_MARKS_SPOT;
          targetAZ += 90;
        } else {
          targetX = get<0>(blueIsolation[0])*X_MARKS_SPOT;
          targetY = get<1>(blueIsolation[0])*X_MARKS_SPOT;
          targetAZ += 90;
        }

        if (targetAZ >= 360){
          targetAZ -= 360;
        }

        phase++;
      } case 5: { // drive to goal
        double angle;
        if(drivePhase == 1 || drivePhase == 2)
          angle = tank.angleBetween(local_map.pos.x, local_map.pos.y, targetX, targetY);
        else
          angle = targetAZ; // target location
        
        if(drive(local_map, targetX, targetY, angle)){
          drivePhase = 1; phase++;
        }

        break;
      } case 6: { // deposit ball
        vector<MAP_OBJECTS> ballsInGoal;
        // Rewrite
        for(MAP_OBJECTS each: local_map.mapobj){
          if(each.classID != 2 && (abs(each.positionX - targetX * GOAL_CONST/GOAL_CONST_BEFORE) < DISTANCE_BUFFER && abs(each.positionY - targetY * GOAL_CONST/GOAL_CONST_BEFORE) < DISTANCE_BUFFER)){
            ballsInGoal.push_back(each);
          }
        }
        sort(ballsInGoal.begin(), ballsInGoal.end(), orderByHeight);
        // fprintf(fp, "%d", ballsInGoal.size());

        tank.drive(50, 0);
        this_thread::sleep_for(1000);

        tank.drive(0, 0);
        if(ballsInGoal.size() == 3){
          ballStor.intake(50);
          this_thread::sleep_for(750);
        }

        ballStor.intake(0);
        ballStor.shoot(50);

        this_thread::sleep_for(500);

        ballStor.shoot(0);
        tank.drive(-50, 0);

        this_thread::sleep_for(1000);

        tank.drive(0, 0);

        if(ballsInGoal.size() == 3){
          if(ballsInGoal[0].classID == TEAM_COLOR){
            phase = 4;
          } else {
            ballStor.shoot(50);
            this_thread::sleep_for(1000);
            ballStor.shoot(0);
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
  tank.drive(50, 0);
  
  this_thread::sleep_for(1000);

  tank.drive(0, 0);
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
  vexcodeInit();

  // thread t1(dashboardTask);
  // Competition.autonomous(autonomousMain);
  // linkA.received("demoMessage", receiveDemo);
  left_drive.spin(directionType::fwd, 50, percentUnits::pct);
  right_drive.spin(directionType::fwd, 50, percentUnits::pct);
  this_thread::sleep_for(500);
  left_drive.spin(directionType::fwd, 0, percentUnits::pct);
  right_drive.spin(directionType::fwd, 0, percentUnits::pct);
  

  // Prevent main from exiting with an infinite loop.
  // print through the controller to the terminal (vexos 1.0.12 is needed)
    // As USB is tied up with Jetson communications we cannot use
    // printf for debug.  If the controller is connected
    // then this can be used as a direct connection to USB on the controller
    // when using VEXcode.
    //

    while(1) {
        // get last map data
        jetson_comms.get_data( &local_map );

        // set our location to be sent to partner robot
        // link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az );

        // request new data    
        // NOTE: This request should only happen in a single task.    
        jetson_comms.request_map();

        // Allow other tasks to run
        this_thread::sleep_for(loop_time);
    }
}
