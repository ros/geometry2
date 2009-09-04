/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**\author Tully Foote/tfoote at willowgarage.com */

#include "tf/transform_broadcaster.h"
#include <cmath>

class SpaceShuttleTracker 
{
public:
  //constructor
  SpaceShuttleTracker() : count(2){}
  //Clean up ros connections
  ~SpaceShuttleTracker() { }
  
  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;


  void sendSpaceShuttlePosition () {
    double period = 60*90;
    double orbit_fraction = fmod(ros::Time::now().toSec(),period)  / period;
    double angle = orbit_fraction * (2 * M_PI);
    double radius = 1e4;
    double x = radius * cos(angle);
    double y = radius * sin(angle);
    if (orbit_fraction > 0.5) //The shuttle is visible half the time from hawaii and half the time from hawaii and half the time from canaveral
    {
      broadcaster.sendTransform(btTransform(btQuaternion(0,0,0), btVector3(x,y,0)), ros::Time::now(), "space_shuttle", "hawaii");
      ROS_INFO("Tracking Shuttle via hawaii");
    }
    else
    {
      broadcaster.sendTransform(btTransform(btQuaternion(0,0,0), btVector3(x,y,0)), ros::Time::now(), "space_shuttle", "canaveral");
      ROS_INFO("Tracking Shuttle via canaveral");
    }
    //note I haven't corrected this "observation" for the difference in position between hawaii and canaveral so the position will be discontinuous if 
    //hawaii and canaveral are not colocated(which they won't be for this example)
  };

  void sendMoonPosition () {
    double period = 60*60*12;
    double orbit_fraction = fmod(ros::Time::now().toSec(),period)  / period;
    double angle = orbit_fraction * (2 * M_PI);
    double radius = 1e5;
    double x = radius * cos(angle);
    double y = radius * sin(angle);
    ROS_INFO("Tracking Moon");
    broadcaster.sendTransform(btTransform(btQuaternion(-angle,0,0), btVector3(x,y,0)), ros::Time::now(), "moon", "earth");
  };
private:
  int count;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"nasa_tracking");
  ros::NodeHandle node;//\todo replace with ros::ok() when 0.5.2 is released

  //Construct/initialize the server
  SpaceShuttleTracker mySpaceShuttleTracker;

  while(node.ok())//Check if a Ctrl-C or other shutdown command has been recieved
  {
    //Send the position of the earth with respect to the sun
    mySpaceShuttleTracker.sendSpaceShuttlePosition();   //Send the shuttle position in relation to tracking stations in hawaii or florida
    mySpaceShuttleTracker.sendMoonPosition();  //Send the position of the moon
    ros::Duration(1.0).sleep();//Sleep for one second
  }

  return 0;
};

