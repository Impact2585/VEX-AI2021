#include "vex.h"
#include "robotMap.h"
#include "tankDrive.h"
#include <cmath>

using namespace vex;
using namespace std;

#define PI 3.14159265
#define DISTANCE_BUFFER 3.0
#define TURNING_BUFFER 3.0

#define turn_kP 0.3 // TO-DO: Tune kP
#define move_kP 0.5 // TO-DO: Tune kP

#define moveConst 45
#define rotateConst 45

FILE *fpp = fopen("/dev/serial2","wb");

float x = 0;
float y = 0;
float az = 0;

tankDrive::tankDrive(){}

void tankDrive::move_left_side(double speed) {
  left_drive.spin(directionType::fwd, speed, percentUnits::pct);
}

void tankDrive::move_right_side(double speed) {
  right_drive.spin(directionType::fwd, speed, percentUnits::pct);
}

void tankDrive::drive(double speed, double turn){
  move_left_side(speed - turn);
  move_right_side(speed + turn);
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
  return max(-100.0, min(error * move_kP, 100.0));
}

bool tankDrive::move(double dist, double targetDist, double heading, double targetHeading, double speedMultiplier){
    // fprintf(fpp, "%.2f %.2f\n", heading, targetHeading);

  if(heading < 0)
    heading += 360;
  if(targetHeading < 0)
    targetHeading += 360;
  
  double power = move_speed(dist, targetDist);
  double turn = turn_speed(heading, targetHeading);
  
  if(abs(targetHeading - heading) > 10){
    drive(0, turn * speedMultiplier);
  } else {
  // fprintf(fpp, "%.2f %.2f\n", heading, targetHeading);
    drive(power * speedMultiplier, turn * speedMultiplier);
  }

  return abs(targetDist - dist) < DISTANCE_BUFFER && abs(targetHeading - heading) < TURNING_BUFFER; // Returns true if we are at the target x,y,ax, false if we have not yet reached the destination
}

bool tankDrive::move(double dist, double targetDist, double heading, double targetHeading){
  return move(dist, targetDist, heading, targetHeading, 0.5);
}

void tankDrive::drive(double dist){ // distance is in inches
  // Do math to figure out new location
  x += dist * cos(az * PI/180);
  y += dist * sin(az * PI/180);
  left_drive.spinFor(directionType::fwd, dist * moveConst, rotationUnits::deg, true);
  right_drive.spinFor(directionType::fwd, dist * moveConst, rotationUnits::deg, true);
}

void tankDrive::rotate(double angle){ // distance is in inches
  az += angle;
  while(az > 360)
    az -= 360;
  while(az < 0)
    az += 360;
  left_drive.spinFor(directionType::fwd, angle * rotateConst, rotationUnits::deg, true);
  right_drive.spinFor(directionType::rev, angle * rotateConst, rotationUnits::deg, true);
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

double tankDrive::angleBetween(double x, double y, double tX, double tY){
  double angle = (atan2(tY - y, tX - x)) * 180 / M_PI;
  if(tY - y < 0){
    angle += 180; 
  }
  if(angle < 0)
  angle += 360;
  return angle;
}