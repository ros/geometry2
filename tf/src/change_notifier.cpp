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

/** \author Tully Foote */

#include "ros/node.h"
#include "tf/transform_listener.h"

/** This is a program to provide notifications of changes of state within tf 
 * It was written for providing an easy way to on demand update a web graphic of 
 * where the robot is located.  It's not designed or recommended for use in live 
 * operation for feedback.  */
int main(int argc, char** argv)
{
  double polling_frequency;
  double translational_update_distance, angular_update_distance;
  std::string source_frame, target_frame, topic_name;

  ros::init(argc, argv);
  ros::Node node("change_notifier", ros::Node::ANONYMOUS_NAME);

  node.param(std::string("~polling_frequency"), polling_frequency, 10.0);
  node.param(std::string("~translational_update_distance"), translational_update_distance, 0.10);
  node.param(std::string("~angular_update_distance"), angular_update_distance, 0.10);

  node.param(std::string("~source_frame"), source_frame, std::string("base_link"));
  node.param(std::string("~target_frame"), target_frame, std::string("map"));
  node.param(std::string("~topic_name"), topic_name, std::string("robot_pose_visualization"));


  tf::TransformListener tfl(node);

  //Advertise the service
  node.advertise<geometry_msgs::PoseStamped>(topic_name, 1);
  

  geometry_msgs::PoseStamped msg;
  tf::Stamped<tf::Pose> pose_in, pose_out, last_sent_pose;
  pose_in = tf::Stamped<tf::Pose> (tf::Pose(tf::Quaternion(0,0,0), 
                                            tf::Vector3(0,0,0)),
                                   ros::Time(), 
                                   source_frame);


  while (node.ok())
  {
    try
    {
      tfl.transformPose(target_frame, pose_in, pose_out);
      if (pose_out.getOrigin().distance(last_sent_pose.getOrigin()) > translational_update_distance ||
          pose_out.getRotation().angle(last_sent_pose.getRotation()) > angular_update_distance )
      {
        last_sent_pose = pose_out;
        tf::poseStampedTFToMsg(pose_out, msg);
        node.publish(topic_name, msg);
      }
      
    }
    catch (tf::TransformException &ex)
    {
      ROS_DEBUG("Exception: %s\n", ex.what());
      continue;
    }
    
    //sleep until next polling
    if (polling_frequency > 0)
      ros::Duration().fromSec(1.0/polling_frequency).sleep();
    
  }


  return 0;
};
