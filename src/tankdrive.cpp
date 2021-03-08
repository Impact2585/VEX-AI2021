#include "vex.h"
#include "robotMap.h"
#include "tankDrive.h"
#include <cmath>

using namespace vex;
using namespace std;

#define PI 3.14159265
#define DISTANCE_BUFFER 1.0
#define TURNING_BUFFER 5.0

double turn_kP = 1.0; // TO-DO: Tune kP
double move_kP = 1.0; // TO-DO: Tune kP

tankDrive::tankDrive() : Motor_Left_1(LEFT_MOTOR1), Motor_Left_2(LEFT_MOTOR2), Motor_Right_1(RIGHT_MOTOR1), Motor_Right_2(RIGHT_MOTOR2) {}

void tankDrive::move_left_side(double speed) {
  Motor_Left_1.setVelocity(speed, velocityUnits::pct);
  Motor_Left_2.setVelocity(speed, velocityUnits::pct);
}

void tankDrive::move_right_side(double speed) {
  Motor_Right_1.setVelocity(speed, velocityUnits::pct);
  Motor_Right_2.setVelocity(speed, velocityUnits::pct);
}

void tankDrive::left_turn(double speed){
  Motor_Left_1.setVelocity(-speed, velocityUnits::pct);
  Motor_Left_2.setVelocity(-speed, velocityUnits::pct);

  Motor_Right_1.setVelocity(speed, velocityUnits::pct);
  Motor_Right_2.setVelocity(speed, velocityUnits::pct);
}

void tankDrive::right_turn(double speed){
  Motor_Left_1.setVelocity(speed, velocityUnits::pct);
  Motor_Left_2.setVelocity(speed, velocityUnits::pct);

  Motor_Right_1.setVelocity(-speed, velocityUnits::pct);
  Motor_Right_2.setVelocity(-speed, velocityUnits::pct);
}

double tankDrive::turn_speed(double heading, double target){ // Returns the motor speed adjustment based on turning
  double rightError = target - heading;
  if(rightError < 0)
    rightError += 360;
  double leftError = 360 - rightError;

  if(rightError < leftError){
    return min(rightError * turn_kP, 100.0);
  } else if (leftError < rightError){
    return -min(leftError * turn_kP, 100.0);
  } else {
    return 0;
  }
}

double tankDrive::move_speed(double dist, double target){ // Returns the motor speed
  double error = target - dist;
  return min(error * move_kP, 100.0);
}

bool tankDrive::move(double dist, double targetDist, double heading, double targetHeading){
  double power = move_speed(dist, targetDist);
  double turn = turn_speed(heading, targetHeading);

  move_left_side(power + turn);
  move_right_side(power - turn);

  return abs(targetDist - dist) > DISTANCE_BUFFER && abs(targetHeading - heading) > TURNING_BUFFER; // Returns true if we are at the target x,y,ax, false if we have not yet reached the destination
}

tuple<pair<double, double>, double> tankDrive::closestJoinHighway(double x, double y){
  double newX;
  double newY;
  double a;

  if (x < -18){
    if (y < -18){
      newX = -18;
      newY = -18;
      a = -(atan2 (-18-x, -18-y) * 180 / PI)+90;
    } else if (-18 <= y && y <= 18){
      newX = -18;
      newY = y;
      a = 90;
    } else if (y > 18){
       newX = -18;
       newY = 18;
       a = -(atan2 (-18-x, 18-y) * 180 / PI)+90;
    }
  } else if (-18 <= x && x<= 18){
    if (y < -18){
      newX = x;
      newY = -18;
      a = 0;
    } else if (-18 <= y && y<= 18){
      if ((y > x && x > 0) || (y > -x && x < 0)){
        newX = x;
        newY = 18;
        a = 0;
      } else if ((y < x && x > 0) || (y > - x && x > 0)){
        newX = 18;
        newY = y;
        a = 90;
      } else if ((y < -x && x > 0) || (y<x && x < 0)){
        newX = x;
        newY = -18;
        a = 180;
      } else if ((y > x && x < 0) || (y < -x && x < 0)){
        newX = -18;
        newY = y;
        a = 270;
      }
    } else if (y> 18){
      newX = x;
      newY = 18;
      a = 180;
    }
  } else if (x > 18){
     if (y < -18){
      newX = 18;
      newY = -18;
      a = -(atan2 (18-x, -18-y) * 180 / PI)+90;
    } else if (-18 <= y && y<= 18){
      newX = 18;
      newY = y;
      a = 270;
    } else if (y> 18){
      newX = 18;
      newY = 18;
      a = -(atan2 (18-x, 18-y) * 180 / PI) + 90;
    }
  }
  return tuple<pair<double, double>, double> {pair<double,double>{newX, newY}, a};
}

tuple<pair<double, double>, double> tankDrive::closestLeaveHighway(double targetX, double targetY){
  return closestJoinHighway(targetX, targetY);
}