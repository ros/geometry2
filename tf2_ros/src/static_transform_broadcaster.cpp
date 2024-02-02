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
#include <mutex>

namespace tf2_ros {

struct StaticTransformBroadcasterImpl {
  ros::NodeHandle node_;  // internal reference to node
  ros::Publisher publisher_;
  tf2_msgs::TFMessage net_message_;  // message comprising all static transforms

  StaticTransformBroadcasterImpl() {
    publisher_ = node_.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);
  }
  StaticTransformBroadcasterImpl(const StaticTransformBroadcasterImpl& other) = delete;

  static std::shared_ptr<StaticTransformBroadcasterImpl> getInstance() {
    static std::mutex mutex;
    static std::weak_ptr<StaticTransformBroadcasterImpl> singleton;

    std::lock_guard<std::mutex> lock(mutex);
    if (singleton.expired()) {  // create a new instance if required
      auto result = std::make_shared<StaticTransformBroadcasterImpl>();
      singleton = result;
      return result;
    }  // otherwise return existing one
    return singleton.lock();
  }
};

StaticTransformBroadcaster::StaticTransformBroadcaster()
{
  impl_ = StaticTransformBroadcasterImpl::getInstance();
};

void StaticTransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  for (const geometry_msgs::TransformStamped& input : msgtf)
  {
    auto predicate = [&input](const geometry_msgs::TransformStamped existing) {
      return input.child_frame_id == existing.child_frame_id;
    };

    auto& transforms = impl_->net_message_.transforms;
    auto existing = std::find_if(transforms.begin(), transforms.end(), predicate);

    if (existing != transforms.end())
      *existing = input;
    else
      transforms.push_back(input);
  }

  impl_->publisher_.publish(impl_->net_message_);
}

}
