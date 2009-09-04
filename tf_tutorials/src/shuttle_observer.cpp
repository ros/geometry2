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

#include "tf/transform_datatypes.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "shuttle_tracker");
  ros::NodeHandle node;

  
  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped> ("object", 10);
  while(node.ok())//Check if a Ctrl-C or other shutdown command has been recieved
  {
    //The shuttle sees something off to the side of it parallel
    tf::Stamped<tf::Pose> object_pose = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(0,0,0), tf::Point(2,2,0)), ros::Time::now(), "space_shuttle");
    geometry_msgs::PoseStamped object_pose_msg;
    tf::poseStampedTFToMsg(object_pose, object_pose_msg);  //This could be constructed directly but the tf/bullet helper functions are easier
    //And reports it
    ROS_INFO("Shuttle saw an object at 2,2,0 at time %.2f", object_pose_msg.header.stamp.toSec());
    pub.publish(object_pose_msg);
    ros::Duration(1.0).sleep();//Sleep for one second
  }

  return 0;
};

