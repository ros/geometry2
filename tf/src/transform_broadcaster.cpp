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


/** \author Tully Foote */


#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

namespace tf {

TransformBroadcaster::TransformBroadcaster()
{
  publisher_ = node_.advertise<tfMessage>("/tf", 100);
  ros::NodeHandle l_nh("~");
  tf_prefix_ = getPrefixParam(l_nh);
};

void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
{
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(msgtf);
  sendTransform(v1);
}

void TransformBroadcaster::sendTransform(const StampedTransform & transform)
{
  std::vector<StampedTransform> v1;
  v1.push_back(transform);
  sendTransform(v1);
} 

void TransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  tfMessage message;
  for (std::vector<geometry_msgs::TransformStamped>::const_iterator it = msgtf.begin(); it != msgtf.end(); ++it)
  {
    message.transforms.push_back(*it);
    //Make sure to resolve anything published
    message.transforms.back().header.frame_id = tf::resolve(tf_prefix_, message.transforms.back().header.frame_id);
    message.transforms.back().child_frame_id = tf::resolve(tf_prefix_, message.transforms.back().child_frame_id);
  }
  publisher_.publish(message);
}

void TransformBroadcaster::sendTransform(const std::vector<StampedTransform> & transforms)
{
  std::vector<geometry_msgs::TransformStamped> msgtfs;
  for (std::vector<StampedTransform>::const_iterator it = transforms.begin(); it != transforms.end(); ++it)
  {
    geometry_msgs::TransformStamped msgtf;
    transformStampedTFToMsg(*it, msgtf);
    msgtfs.push_back(msgtf);

  }
  sendTransform(msgtfs);
} 
  



}


