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

#define STOP_BEFORE 12.0
#define GOAL_CONST 68.0

// 0 is red team, 1 is blue team. Change the program run during competition for rounds. 1 = red team, 2 = blue team
#define TEAM_COLOR 0
// This is the manager robot. THE MANAGER ROBOT IS  PLACED TO THE NORTH OF THE WORKER ROBOT 
// ON THE RED TEAM, AND SOUTH OF THE WORKER ROBOT ON THE BLUE TEAM.
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

int32_t loop_time = 66;
static MAP_RECORD local_map;

FILE *fp = fopen("/dev/serial2","wb");

bool orderByHeight (fifo_object_box i, fifo_object_box j) {
  return i.y > j.y;
}

double dist(double ax, double ay, double bx, double by){
  return sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

void score(){
  tank.drive(20, 0);
  this_thread::sleep_for(1000);
  tank.drive(0, 0);
  ballStor.shoot(100);
  this_thread::sleep_for(1000);
  ballStor.shoot(0);
  tank.drive(-20, 0);
  this_thread::sleep_for(1000);
  tank.drive(0, 0);
}

void intake(){
  tank.drive(20, 0);
  this_thread::sleep_for(1000);
  tank.drive(0, 0);
  ballStor.intake(100);
  this_thread::sleep_for(1000);
  ballStor.intake(0);
  tank.drive(-20, 0);
  this_thread::sleep_for(1000);
  tank.drive(0, 0);
}

int play(bool isolation) {
  int curGoal = -1;
  tuple<float, float> goals[6];
  if(TEAM_COLOR == 0){
    tuple<float, float> g [] = {tuple<float, float>(-1,1), tuple<float, float>(-1, 0), tuple<float, float>(0,1), tuple<float, float>(0, 0), tuple<float, float>(1,1), tuple<float, float>(1, 0)};
    for(int i = 0; i < 6; i++){
      goals[i] = g[i];
    }
  } else {
    tuple<float, float> g [] = {tuple<float, float>(-1,-1), tuple<float, float>(-1, 0), tuple<float, float>(0,-1), tuple<float, float>(0, 0), tuple<float, float>(1,-1), tuple<float, float>(1, 0)};
    for(int i = 0; i < 6; i++){
      goals[i] = g[i];
    }
  }

  while(2585 > 285){
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
      
      while(!tank.move(STOP_BEFORE, dist(targetX, targetY, local_map.pos.x, local_map.pos.y), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, targetX, targetY), 0.5)){
        this_thread::sleep_for(loop_time);
      }
      while(!tank.move(0, 0, local_map.pos.az, targetAZ, 0.5)){
        this_thread::sleep_for(loop_time);
      }

      vector<fifo_object_box> ballsInGoal;
      for(fifo_object_box each: local_map.boxobj){
        if(each.classID != 2 && (each.x != 0.0 || each.y != 0.0)){
          ballsInGoal.push_back(each);
        }
      }
      sort(ballsInGoal.begin(), ballsInGoal.end(), orderByHeight);
      for(fifo_object_box each: ballsInGoal){
        fprintf(fp, "%d %d\n", each.x, each.y);
      }

      tank.drive(30, 0);
      this_thread::sleep_for(500);
      tank.drive(0, 0);
      while(ballsInGoal.size() > 0){
        if(ballsInGoal.back().classID == TEAM_COLOR){
          
          ballStor.intake(50);
          ballStor.shoot(50);

          this_thread::sleep_for(500);

          ballStor.intake(0);
          ballStor.shoot(0);
        } else {
          ballStor.intake(50);
          this_thread::sleep_for(500);
          ballStor.intake(0);


          tank.drive(-30, 0);
          this_thread::sleep_for(500);
          tank.drive(0, 30);
          this_thread::sleep_for(500);
          tank.drive(0, 0);
          ballStor.intake(-50);
          this_thread::sleep_for(500);
          ballStor.intake(0);
          tank.drive(0, -30);
          this_thread::sleep_for(500);
          tank.drive(30, 0);
          this_thread::sleep_for(500);
          tank.drive(0, 0);
        }
        ballsInGoal.pop_back();
      }
      tank.drive(-30, 0);
      this_thread::sleep_for(500);
      tank.drive(0, 0);
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
  this_thread::sleep_for(250);
  tank.drive(0, 0);

  if (TEAM_COLOR == 0){
    // Red: Manager on top
    if (manager_robot){
      // Drive to middle left goal
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, -GOAL_CONST, 0), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, -GOAL_CONST, 0), 0.5)){
        this_thread::sleep_for(loop_time);
      }
      
      // Deposit ball
      score();

      ballStor.intake(50);
      // Move towards top left goal and intake ball
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, -GOAL_CONST, GOAL_CONST), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, -GOAL_CONST, GOAL_CONST), 0.5)){
        this_thread::sleep_for(loop_time);
      }
      
      score();
      // Turn towards center ball on midline

      ballStor.intake(50);
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, 0, 36), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, 0, 36), 0.50)){
        this_thread::sleep_for(loop_time);
      }
      ballStor.intake(0);

      intake();
      // Turn towards middle goal

      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, 0, 0), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, 0, 0), 0.25)){
        this_thread::sleep_for(loop_time);
      }
      score();

    // Red: worker on Bottom
    } else {
      // Turn towards bottom left goal
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, -GOAL_CONST, -GOAL_CONST), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, -GOAL_CONST, -GOAL_CONST), 0.5)){
        this_thread::sleep_for(loop_time);
      }
      
      // Deposit ball
      score();
    }
  }
  // Blue
  else if (TEAM_COLOR == 1){
    // Blue: Manager (on bottom)
    if (manager_robot){
        // Drive to middle right goal
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, GOAL_CONST, 0), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, GOAL_CONST, 0), 0.5)){
        this_thread::sleep_for(loop_time);
      }
      
      // Deposit ball
      score();

      ballStor.intake(50);
      // Move towards bottom right goal and intake ball
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, GOAL_CONST, -GOAL_CONST), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, GOAL_CONST, -GOAL_CONST), 0.5)){
        this_thread::sleep_for(loop_time);
      }
      
      score();
      // Turn towards center ball on midline

      ballStor.intake(50);
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, 0, -36), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, 0, -36), 0.50)){
        this_thread::sleep_for(loop_time);
      }
      ballStor.intake(0);

      intake();
      // Turn towards middle goal

      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, 0, 0), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, 0, 0), 0.25)){
        this_thread::sleep_for(loop_time);
      }
      score();
    }else{
      // Turn towards top right goal
      while(!tank.move(STOP_BEFORE, dist(local_map.pos.x, local_map.pos.y, GOAL_CONST, GOAL_CONST), local_map.pos.az, tank.angleBetween(local_map.pos.x, local_map.pos.y, GOAL_CONST, GOAL_CONST), 0.5)){
        this_thread::sleep_for(loop_time);
      }
      
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

        // Allow other tasks to run
        this_thread::sleep_for(loop_time);
    }
}
