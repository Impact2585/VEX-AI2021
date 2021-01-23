/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2020 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     dashboard.cpp                                               */
/*    Author:     James Pearman                                               */
/*    Created:    20 August 2020                                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

//
// Display various useful information about the Jetson
//
static void
dashboardJetson( int ox, int oy, int width, int height ) {
  static int32_t     last_data = 0;
  static int32_t     last_packets = 0;
  static int32_t     total_data = 0;
  static int32_t     total_packets = 0;
  static uint32_t    update_time = 0;
  static MAP_RECORD  local_map;
  color grey = vex::color(0x404040);

  Brain.Screen.setClipRegion( ox, oy, width, height);
  Brain.Screen.setFont( mono15 );
  // border and titlebar
  Brain.Screen.setPenColor( yellow );
  Brain.Screen.drawRectangle(ox, oy, width, height, black );
  Brain.Screen.drawRectangle( ox, oy, width, 20, grey );

  Brain.Screen.setPenColor( yellow );
  Brain.Screen.setFillColor( grey );
  Brain.Screen.printAt(ox + 10, oy + 15, "Jetson" );
  oy += 20;
  
  Brain.Screen.setPenColor( white );
  Brain.Screen.setFillColor( black );
  
  // get last map data
  jetson_comms.get_data( &local_map );

  Brain.Screen.printAt( ox + 10, oy += 15, "Packets   %d", jetson_comms.get_packets() );
  Brain.Screen.printAt( ox + 10, oy += 15, "Errors    %d", jetson_comms.get_errors() );
  Brain.Screen.printAt( ox + 10, oy += 15, "Timeouts  %d", jetson_comms.get_timeouts() );
  Brain.Screen.printAt( ox + 10, oy += 15, "data/sec  %d             ", total_data );
  Brain.Screen.printAt( ox + 10, oy += 15, "pkts/sec  %d             ", total_packets );
  Brain.Screen.printAt( ox + 10, oy += 15, "boxnum    %d", local_map.boxnum );
  Brain.Screen.printAt( ox + 10, oy += 15, "mapnum    %d", local_map.mapnum );

  // once per second, update data rate stats
  if( Brain.Timer.system() > update_time ) {
    update_time = Brain.Timer.system() + 1000;
    total_data = jetson_comms.get_total() - last_data;
    total_packets = jetson_comms.get_packets() - last_packets;
    last_data = jetson_comms.get_total();
    last_packets = jetson_comms.get_packets();
  }
  
  Brain.Screen.setFont( mono12 );
  for(int i=0;i<4;i++ ) {
    if( i < local_map.boxnum ) {
      Brain.Screen.printAt( ox + 10, oy += 12, "box %d: x:%4d y:%4d w:%4d h:%4d d:%.1f",i,
                           local_map.boxobj[i].x,
                           local_map.boxobj[i].y,
                           local_map.boxobj[i].width,
                           local_map.boxobj[i].height,
                           local_map.boxobj[i].depth );
    }
    else {
      Brain.Screen.printAt( ox + 10, oy += 12, "---");
    }
  }
  for(int i=0;i<4;i++ ) {
    if( i < local_map.mapnum ) {
      Brain.Screen.printAt( ox + 10, oy += 12, "map %d: a:%4d c:%4d 0:%.2f 1:%.2f 2:%.1f",i,
                           local_map.mapobj[i].age,
                           local_map.mapobj[i].classID,
                           local_map.mapobj[i].p[0],
                           local_map.mapobj[i].p[1],
                           local_map.mapobj[i].p[2]);
    }
    else {
      Brain.Screen.printAt( ox + 10, oy += 12, "---");
    }
  }

}

//
// Task to update screen with status
//
int
dashboardTask() {
  while(true) {
    // status
    dashboardJetson(    0, 0, 280, 240 );
    // draw, at 30Hz
    Brain.Screen.render();
    this_thread::sleep_for(16);
  }
  return 0;
}