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

namespace tf {

TransformBroadcaster::TransformBroadcaster()
{
  publisher_ = node_.advertise<tfMessage>("/tf_message", 100);
  node_.param(std::string("~tf_prefix"),tf_prefix_, std::string(""));
};

void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
{
  tfMessage message;
  message.transforms.push_back(msgtf);
  publisher_.publish(message);
}

void TransformBroadcaster::sendTransform(const Stamped<Transform> & transform)
{
  geometry_msgs::TransformStamped msgtf;
  transformStampedTFToMsg(transform, msgtf);
  msgtf.header.frame_id = tf::remap(tf_prefix_, msgtf.header.frame_id);
  msgtf.child_frame_id = tf::remap(tf_prefix_, msgtf.child_frame_id);
  sendTransform(msgtf);
} 
  

void TransformBroadcaster::sendTransform(const Transform & transform, const ros::Time& time, const std::string& frame_id, const std::string& parent_id)
{
  tfMessage message;
  geometry_msgs::TransformStamped msgtf;
  msgtf.header.stamp = time;
  msgtf.header.frame_id = tf::remap(tf_prefix_, parent_id);
  msgtf.child_frame_id = tf::remap(tf_prefix_, frame_id);
  transformTFToMsg(transform, msgtf.transform);
  message.transforms.push_back(msgtf);
  publisher_.publish(message);
}


}


