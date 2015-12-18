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

#ifndef TF2_ROS_TRANSFORMLISTENER_H
#define TF2_ROS_TRANSFORMLISTENER_H

#include <thread>
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"


namespace tf2_ros{

class TransformListener 
{

public:
  /**@brief Constructor for transform listener */
  TransformListener(tf2::BufferCore& buffer, bool spin_thread = true);
  TransformListener(tf2::BufferCore& buffer, rclcpp::node::Node::SharedPtr nh, bool spin_thread = true);

  ~TransformListener();

private:

  /// Initialize this transform listener, subscribing, advertising services, etc.
  void init();
  void initThread();

  /// Callback function for ros message subscriptoin
  void subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void static_subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void subscription_callback_impl(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

  // ros::CallbackQueue tf_message_callback_queue_;
  std::thread* dedicated_listener_thread_;
  rclcpp::node::Node::SharedPtr node_;
  rclcpp::subscription::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::subscription::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;
  tf2::BufferCore& buffer_;
  bool using_dedicated_thread_;
  tf2::TimePoint last_update_;
 
  void dedicatedListenerThread()
  {
    while (using_dedicated_thread_)
    {
      break;
      //TODO(tfoote) reenable callback queue processing 
      //tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01));
    }
  };

};
}

#endif //TF_TRANSFORMLISTENER_H
