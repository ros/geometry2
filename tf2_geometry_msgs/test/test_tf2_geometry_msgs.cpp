/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Wim Meeussen */


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_cpp/transform_listener.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bla");
  ros::NodeHandle n;
  ros::Duration(2.0).sleep();
  tf2::Buffer tf_buffer;
  tf2::TransformListener listener(tf_buffer);

  // Vector3Stamped
  geometry_msgs::Vector3Stamped v1, res;
  v1.vector.x = 1;
  v1.vector.y = 2;
  v1.vector.z = 3;
  v1.header.stamp = ros::Time::now();
  v1.header.frame_id = "r_forearm_link";
  
  std::cout << tf_buffer.transform(v1, "head_pan_link", ros::Time(), "base_link", ros::Duration(3.0)) << std::endl;
  std::cout << tf_buffer.transform(v1, "torso_lift_link", ros::Duration(3.0)) << std::endl;


  // PointStamped
  geometry_msgs::PointStamped v2;
  v2.point.x = 1;
  v2.point.y = 2;
  v2.point.z = 3;
  v2.header.stamp = ros::Time::now();
  v2.header.frame_id = "wimpie";

  tf_buffer.transform(v2, "blo", ros::Time::now(), "fixed_frame", ros::Duration(3.0));
  tf_buffer.transform(v2, "blo", ros::Duration(3.0));

  // PoseStamped
  geometry_msgs::PoseStamped v3;
  v3.pose.position.x = 1;
  v3.pose.position.y = 2;
  v3.pose.position.z = 3;
  v3.pose.orientation.x = 1;
  v3.header.stamp = ros::Time::now();
  v3.header.frame_id = "wimpie";

  tf_buffer.transform(v3, "blo", ros::Time::now(), "fixed_frame", ros::Duration(3.0));
  tf_buffer.transform(v3, "blo", ros::Duration(3.0));


  return 0;
}
