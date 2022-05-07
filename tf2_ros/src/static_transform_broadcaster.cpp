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
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <algorithm>

namespace tf2_ros {

StaticTransformBroadcaster::StaticTransformBroadcaster()
{
  publisher_ = node_.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);
};

StaticTransformBroadcaster::StaticTransformBroadcaster(const ros::NodeHandle& node) : node_(node)
{
  publisher_ = node_.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);
};

void StaticTransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  for (const geometry_msgs::TransformStamped& input : msgtf)
  {
    auto predicate = [&input](const geometry_msgs::TransformStamped existing) {
      return input.child_frame_id == existing.child_frame_id;
    };
    auto existing = std::find_if(net_message_.transforms.begin(), net_message_.transforms.end(), predicate);

    if (existing != net_message_.transforms.end())
      *existing = input;
    else
      net_message_.transforms.push_back(input);
  }

  publisher_.publish(net_message_);
}

}
