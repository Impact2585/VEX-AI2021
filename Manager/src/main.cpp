#include "vex.h"
#include "tankDrive.h"
#include "ballStorage.h"
#include "robotMap.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdio.h>
using namespace vex;
using namespace std;

#define STOP_BEFORE 48.0
#define GOAL_CONST 72.0

// 0 is red team, 1 is blue team. Change the program run during competition for rounds. 1 = red team, 2 = blue team
#define TEAM_COLOR 0
// This is the manager robot. THE MANAGER ROBOT IS  PLACED TO THE NORTH OF THE WORKER ROBOT 
// ON THE RED TEAM, AND SOUTH OF THE WORKER ROBOT ON THE BLUE TEAM.
#define manager_robot true
#define loop_time 66

// A global instance of competition
competition Competition;

// AI Jetson Nano
ai::jetson  jetson_comms;
// #pragma message("building for the manager")
ai::robot_link       link( VEX_LINK, "robot_32456_1", linkType::manager );

// define your global instances of motors and other devices here
static tankDrive tank;
static ballStorage ballStor;

static MAP_RECORD local_map;

FILE *fp = fopen("/dev/serial2","wb");

bool orderByHeight (fifo_object_box i, fifo_object_box j) {
  return i.y > j.y;
}

double dist(double ax, double ay, double bx, double by){
  return sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

void score(){
  //tank.drive(8);
  ballStor.shoot(100);
  this_thread::sleep_for(1000);
  ballStor.shoot(0);
  tank.drive(-8);
}

void intake(){
  tank.drive(8);
  ballStor.intake(100);
  this_thread::sleep_for(1000);
  ballStor.intake(0);
  tank.drive(-8);
}

int play(bool isolation) {
  int curGoal = -1;
  tuple<float, float> goals[6];
  if(TEAM_COLOR == 0){
    tuple<float, float> g [] = {tuple<float, float>(0,0), tuple<float, float>(0, 1), tuple<float, float>(1,1), tuple<float, float>(1, 0), tuple<float, float>(1, -1), tuple<float, float>(0, -1)};
    for(int i = 0; i < 6; i++){
      goals[i] = g[i];
    }
  } else {
    tuple<float, float> g [] = {tuple<float, float>(0, 0), tuple<float, float>(0, -1), tuple<float, float>(-1, -1), tuple<float, float>(-1, 0), tuple<float, float>(-1,1), tuple<float, float>(0, 1)};
    for(int i = 0; i < 6; i++){
      goals[i] = g[i];
    }
  }

  while(2585 > 285){
    this_thread::sleep_for(2000);
    curGoal++;
    if(curGoal >= sizeof(goals)/sizeof(goals[0]))
      curGoal = 0;
    double targetX = GOAL_CONST * get<0>(goals[curGoal]);
    double targetY = GOAL_CONST * get<1>(goals[curGoal]);
    double targetAZ;
    if(curGoal % 2 == 0)
      targetAZ = 0;
    else
      targetAZ = 180;

    while(dist(tank.x, tank.y, targetX, targetY) - STOP_BEFORE > 12){
      turn:
      tank.rotate(-tank.az + tank.angleBetween(tank.x, tank.y, targetX, targetY));
      fprintf(fp, "2: %f %f %f %f %f %f\n", tank.x, tank.y, tank.az, targetX, targetY, tank.angleBetween(tank.x, tank.y, targetX, targetY));
      // Drive to scoring position
      if(abs(-tank.az + tank.angleBetween(tank.x, tank.y, targetX, targetY)) > 10)
        goto turn;
      drive:
      float d = min(dist(tank.x, tank.y, targetX, targetY) - STOP_BEFORE, 36.0);
      tank.drive(d);
    }
    turn2:
    tank.rotate(-tank.az + tank.angleBetween(tank.x, tank.y, targetX, targetY));
    if(abs(-tank.az + tank.angleBetween(tank.x, tank.y, targetX, targetY)) > 8)
        goto turn2;

    vector<fifo_object_box> ballsInGoal;
    for(fifo_object_box each: local_map.boxobj){
      if(each.classID != 2 && (each.x != 0.0 || each.y != 0.0)){
        ballsInGoal.push_back(each);
      }
    }
    while(ballsInGoal.size() > 0){
      sort(ballsInGoal.begin(), ballsInGoal.end(), orderByHeight);
      for(fifo_object_box each: ballsInGoal){
        fprintf(fp, "%d %d %d\n" , each.x, each.y, each.classID);
      }

      if(ballsInGoal.front().classID == TEAM_COLOR)
        break;

      tank.rotate(-tank.az + tank.angleBetween(tank.x, tank.y, targetX, targetY));

      if(ballsInGoal.back().classID == TEAM_COLOR){
        tank.driveTime(2000, 50);
        ballStor.intake(100);

        if(ballsInGoal.size() == 3){
          this_thread::sleep_for(1000);
        } else {
          this_thread::sleep_for(400);
        }
        ballStor.intake(0);
        ballStor.shoot(100);

        this_thread::sleep_for(2000);
        ballStor.shoot(0);

        this_thread::sleep_for(500);
        tank.driveTime(1250, -50);
      } else {
        tank.driveTime(2000, 50);
        ballStor.intake(100);
        if(ballsInGoal.size() == 3){
          this_thread::sleep_for(1000);
        } else {
          this_thread::sleep_for(400);
        }
        ballStor.intake(0);


        tank.driveTime(1250, -50);
        tank.rotate(90);
        ballStor.intake(-100);
        this_thread::sleep_for(500);
        ballStor.intake(0);
        tank.rotate(-90);
      }
      ballsInGoal.clear();
      for(fifo_object_box each: local_map.boxobj){
        if(each.classID != 2 && (each.x != 0.0 || each.y != 0.0)){
          ballsInGoal.push_back(each);
        }
      }
    }

    tank.drive(-24);
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
  //tank.drive(20);

  if (TEAM_COLOR == 0){
    // Red: Manager on top
    if (manager_robot){
      tank.drive(-50);
      tank.rotate(-45);
      tank.drive(28);
      tank.drive(500, 100);
      score();

      tank.drive(-30);
      tank.rotate(58);
      tank.rotate(-tank.az + tank.angleBetween(tank.x, tank.y, -GOAL_CONST, GOAL_CONST));
      ballStor.intake(100);
      tank.drive(60);
      
      ballStor.intake(0);
      tank.drive(500, 100);
      score();
      //tank.rotate(-tank.az);
      //tank.rotate(tank.angleBetween(tank.x, tank.y, -36, 36));

      //drive to (-36, 36)
      //tank.drive(dist(tank.x, tank.y, -36, 36));
      tank.drive(24);
      tank.rotate(315);
      tank.drive(-20);
      //now we're there, turn toward ball/goal at top left
      tank.rotate(tank.angleBetween(tank.x, tank.y, -72, 72));

      //drive to the bottom right corner of the top left square
      tank.drive(dist(tank.x, tank.y, -48, 48));

      //intake the ball there
      intake();

      //move a little forward, then score
      tank.drive(2);
      score();
      tank.rotate(-tank.az);
tank.rotate(tank.angleBetween(tank.x, tank.y, 0, 36));
tank.drive(dist(tank.x, tank.y, 0, 36));
intake();
 tank.rotate(-tank.az);
tank.rotate(tank.angleBetween(tank.x, tank.y, 0, 0));
tank.drive(dist(tank.x, tank.y, 0, 0));
score();
    // Red: worker on Bottom
    } else {
      // Deposit ball
      score();
    }
  }
  // Blue
  else if (TEAM_COLOR == 1){
    // Blue: Manager (on bottom)
    if (manager_robot){
      tank.drive(-50);
      tank.rotate(-45);
      tank.drive(28);
      tank.drive(500, 100);
      score();

      tank.drive(-30);
      tank.rotate(58);
      tank.rotate(-tank.az + tank.angleBetween(tank.x, tank.y, GOAL_CONST, -GOAL_CONST));
      ballStor.intake(100);
      tank.drive(60);
      
      ballStor.intake(0);
      tank.drive(500, 100);
      score();
      //tank.rotate(-tank.az);
      //tank.rotate(tank.angleBetween(tank.x, tank.y, -36, 36));

      //drive to (-36, 36)
      //tank.drive(dist(tank.x, tank.y, -36, 36));
      tank.drive(24);
      tank.rotate(315);
      tank.drive(-20);
      //now we're there, turn toward ball/goal at top left
      tank.rotate(tank.angleBetween(tank.x, tank.y, -72, 72));

      //drive to the bottom right corner of the top left square
      tank.drive(dist(tank.x, tank.y, -48, 48));

      //intake the ball there
      intake();

      //move a little forward, then score
      tank.drive(2);
      score();
      tank.rotate(-tank.az);
tank.rotate(tank.angleBetween(tank.x, tank.y, 0, 36));
tank.drive(dist(tank.x, tank.y, 0, 36));
intake();
 tank.rotate(-tank.az);
tank.rotate(tank.angleBetween(tank.x, tank.y, 0, 0));
tank.drive(dist(tank.x, tank.y, 0, 0));
score();
    // Red: worker on Bottom
    } else {
      // Deposit ball
      score();
    }
  }
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
  thread t1(dashboardTask);
  Competition.autonomous(autonomousMain);
  float lastX = 0;
  float lastY = 0;
  float lastAz = 0;
  // linkA.received("demoMessage", receiveDemo);
  // Prevent main from exiting with an infinite loop.
  // print through the controller to the terminal (vexos 1.0.12 is needed)
    // As USB is tied up with Jetson communications we cannot use
    // printf for debug.  If the controller is connected
    // then this can be used as a direct connection to USB on the controller
    // when using VEXcode.
    while(1) {
        // get last map data
        // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);
        jetson_comms.get_data( &local_map );
        // set our location to be sent to partner robot
        // link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az );
        // request new data    
        // NOTE: This request should only happen in a single task.    
        jetson_comms.request_map();

        // fprintf(fp, "ROBOT LOCATION: " );
        // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);
        // fprintf(fp, "MAP OBJECTS: \n");
        // for(MAP_OBJECTS each: local_map.mapobj){
        //   fprintf(fp, "%ld %ld %.2f %.2f %.2f\n", each.age, each.classID, each.positionX/25.4, each.positionY/25.4, each.positionZ);
        // }
        // fprintf(fp, "\n");

        // Update the current position, if it has changed
        if(local_map.pos.x != lastX || local_map.pos.y != lastY || local_map.pos.az != lastAz){
          tank.x = local_map.pos.x;
          tank.y = local_map.pos.y;
          tank.az = local_map.pos.az;
          while(tank.az < 0)
            tank.az += 360;
          lastX = local_map.pos.x;
          lastY = local_map.pos.y;
          lastAz = local_map.pos.az;
        }

        // Allow other tasks to run
        this_thread::sleep_for(loop_time);
    }
}
