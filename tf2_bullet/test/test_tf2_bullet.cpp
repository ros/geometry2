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


#include <tf2_bullet/tf2_bullet.h>
#include <tf2_cpp/transform_listener.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bla");
  ros::NodeHandle n;
  tf2::Buffer tf_buffer;
  tf2::TransformListener listener(tf_buffer);
  ros::Duration(2.0).sleep();

  // btVector3
  btVector3 v1(1,2,3);
  std::cout << tf_buffer.transform(tf2::Stamped<btVector3>(v1, ros::Time::now(), "head_pan_link"),
				   "r_forearm_link", ros::Time(),
				   "base_link", ros::Duration(3.0)) << std::endl;

  std::cout << tf_buffer.transform(tf2::Stamped<btVector3>(v1, ros::Time(), "head_pan_link"), 
				   "torso_lift_link", ros::Duration(3.0)) << std::endl;

  // btTransform
  btTransform t1(btQuaternion(1,0,0,0));
  std::cout << tf_buffer.transform(tf2::Stamped<btTransform>(t1, ros::Time::now(), "head_pan_link"),
				   "r_forearm_link", ros::Time(),
				   "base_link", ros::Duration(3.0)).getOrigin().getY() << std::endl;

  std::cout << tf_buffer.transform(tf2::Stamped<btTransform>(t1, ros::Time(), "head_pan_link"), 
				   "torso_lift_link", ros::Duration(3.0)).getOrigin().getY() << std::endl;

  return 0;
}
