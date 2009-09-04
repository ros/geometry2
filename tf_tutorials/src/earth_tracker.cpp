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
#include "ros/ros.h"
#include <cmath>

class EarthTracker 
{
public:
  //constructor
  EarthTracker(): count(2){};
  //Clean up ros connections
  ~EarthTracker() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;


  // A function to call to send data periodically
  void sendEarthPosition () {
    double year_fraction = fmod(ros::Time::now().toSec(), (60.0*60*24*365)) / (60.0*60*24*365);
    double seasonal_angle = year_fraction  * (2 * M_PI);
    double day_fraction = fmod(ros::Time::now().toSec(),(60.0*60*24))  / (60.0*60*24);
    double angle = day_fraction * (2 * M_PI);
    double radius = 1e6;
    double x = radius * cos(seasonal_angle);
    double y = radius * sin(seasonal_angle);
    broadcaster.sendTransform(btTransform(btQuaternion(angle,0,0), btVector3(x,y,0)), ros::Time::now(), "earth", "sun");
  };

private:
  int count;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "earth_tracker");

  //Construct/initialize the server
  EarthTracker myEarthTracker;

  ros::NodeHandle node; //\todo replace with ros::ok() after 0.5.2 release
  while(node.ok())//Check if a Ctrl-C or other shutdown command has been recieved
  {
    //Send the position of the earth with respect to the sun
    myEarthTracker.sendEarthPosition();  
    ros::Duration(1.0).sleep();//Sleep for one second
  }

  return 0;
};

