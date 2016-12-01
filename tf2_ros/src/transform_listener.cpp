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

#include "tf2_ros/transform_listener.h"


using namespace tf2_ros;

//TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_INFO printf
#define ROS_WARN printf

TransformListener::TransformListener(tf2::BufferCore& buffer, bool spin_thread):
  dedicated_listener_thread_(NULL), buffer_(buffer), using_dedicated_thread_(false)
{
  //TODO(tfoote)make this anonymous
  node_ = rclcpp::node::Node::make_shared("transform_listener_impl");
  init();
  if (spin_thread)
    initThread();
}

TransformListener::TransformListener(tf2::BufferCore& buffer, rclcpp::node::Node::SharedPtr nh, bool spin_thread)
: dedicated_listener_thread_(NULL)
, node_(nh)
, buffer_(buffer)
, using_dedicated_thread_(false)
{
  init();
  if (spin_thread)
    initThread();
}


TransformListener::~TransformListener()
{
  using_dedicated_thread_ = false;
  if (dedicated_listener_thread_)
  {
    dedicated_listener_thread_->join();
    delete dedicated_listener_thread_;
  }
}

void test_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
  return;
}

void TransformListener::init()
{
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 100;
  std::function<void(const tf2_msgs::msg::TFMessage::SharedPtr)> standard_callback = std::bind(&TransformListener::subscription_callback, this, std::placeholders::_1);
  message_subscription_tf_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf", standard_callback, custom_qos_profile);
  std::function<void(const tf2_msgs::msg::TFMessage::SharedPtr)> static_callback = std::bind(&TransformListener::static_subscription_callback, this, std::placeholders::_1);
  message_subscription_tf_static_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf_static", static_callback, custom_qos_profile);
}

void TransformListener::initThread()
{
  using_dedicated_thread_ = true;
  // This lambda is required because `std::thread` cannot infer the correct
  // rclcpp::spin, since there are more than one versions of it (overloaded).
  // see: http://stackoverflow.com/a/27389714/671658
  // I (wjwwood) chose to use the lamda rather than the static cast solution.
  auto run_func = [](rclcpp::node::Node::SharedPtr node) {
    return rclcpp::spin(node);
  };
  dedicated_listener_thread_ = new std::thread(run_func, node_);
  //Tell the buffer we have a dedicated thread to enable timeouts
  buffer_.setUsingDedicatedThread(true);
}



void TransformListener::subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  subscription_callback_impl(msg, false);
}
void TransformListener::static_subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  subscription_callback_impl(msg, true);
}

void TransformListener::subscription_callback_impl(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static)
{
  auto now = tf2::get_now();
  if(now < last_update_){
    ROS_WARN("Detected jump back in time. Clearing TF buffer.");
    buffer_.clear();
  }
  last_update_ = now;



  const tf2_msgs::msg::TFMessage& msg_in = *msg;
  //TODO(tfoote) find a way to get the authority
  std::string authority = "Authority undetectable"; //msg_evt.getPublisherName(); // lookup the authority
  for (unsigned int i = 0; i < msg_in.transforms.size(); i++)
  {
    try
    {
      buffer_.setTransform(msg_in.transforms[i], authority, is_static);
    }
    
    catch (tf2::TransformException& ex)
    {
      ///\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", msg_in.transforms[i].child_frame_id.c_str(), msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
};
