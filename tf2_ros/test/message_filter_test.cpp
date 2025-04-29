/*
 * Copyright (c) 2014, Open Source Robotics Foundation
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

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gtest/gtest.h>
#include <thread>
#include <chrono>


void spin_for_a_second()
{
  ros::spinOnce();
  for (uint8_t i = 0; i < 10; ++i)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    ros::spinOnce();
  }
}

bool filter_callback_fired = false;
void filter_callback(const geometry_msgs::PointStamped& msg)
{
  filter_callback_fired = true;
}

TEST(tf2_ros_message_filter, multiple_frames_and_time_tolerance)
{
  ros::NodeHandle nh;
  message_filters::Subscriber<geometry_msgs::PointStamped> sub;
  sub.subscribe(nh, "point", 10);

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tfl(buffer);
  tf2_ros::MessageFilter<geometry_msgs::PointStamped> filter(buffer, "map", 10, nh);
  filter.connectInput(sub);
  filter.registerCallback(&filter_callback);
  // Register multiple target frames
  std::vector<std::string> frames;
  frames.push_back("odom");
  frames.push_back("map");
  filter.setTargetFrames(frames);
  // Set a non-zero time tolerance
  filter.setTolerance(ros::Duration(1, 0));

  // Publish static transforms so the frame transformations will always be valid
  tf2_ros::StaticTransformBroadcaster tfb;
  geometry_msgs::TransformStamped map_to_odom;
  map_to_odom.header.stamp = ros::Time(0, 0);
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.x = 0.0;
  map_to_odom.transform.translation.y = 0.0;
  map_to_odom.transform.translation.z = 0.0;
  map_to_odom.transform.rotation.x = 0.0;
  map_to_odom.transform.rotation.y = 0.0;
  map_to_odom.transform.rotation.z = 0.0;
  map_to_odom.transform.rotation.w = 1.0;
  tfb.sendTransform(map_to_odom);

  geometry_msgs::TransformStamped odom_to_base;
  odom_to_base.header.stamp = ros::Time(0, 0);
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "base";
  odom_to_base.transform.translation.x = 0.0;
  odom_to_base.transform.translation.y = 0.0;
  odom_to_base.transform.translation.z = 0.0;
  odom_to_base.transform.rotation.x = 0.0;
  odom_to_base.transform.rotation.y = 0.0;
  odom_to_base.transform.rotation.z = 0.0;
  odom_to_base.transform.rotation.w = 1.0;
  tfb.sendTransform(odom_to_base);

  // Publish a Point message in the "base" frame
  ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("point", 10);
  geometry_msgs::PointStamped point;
  point.header.stamp = ros::Time::now();
  point.header.frame_id = "base";
  pub.publish(point);

  // make sure it arrives
  spin_for_a_second();

  // The filter callback should have been fired because all required transforms are available
  ASSERT_TRUE(filter_callback_fired);
}

template <class M>
class MessageGenerator : public message_filters::SimpleFilter<M>
{
public:
  template <typename F>
  void connectInput(F &)
  {
  }

  void add(const ros::MessageEvent<M const> &)
  {
  }

  void generate(const std::string &frame_id, const ros::Time &time)
  {
    auto msg = boost::make_shared<M>();
    msg->header.frame_id = frame_id;
    msg->header.stamp = time;
    this->signalMessage(msg);
  }
};

class MessageFilterFixture : public ::testing::TestWithParam<bool>
{
  using M = geometry_msgs::PointStamped;

protected:
  tf2_ros::Buffer buffer;
  MessageGenerator<M> source;
  std::list<tf2_ros::MessageFilter<M>> filters;
  bool run = true;

  struct Sink
  {
    std::string name_;
    int delay_;

    Sink(const std::string &name, int delay = 0) : name_(name), delay_(delay) {}
    void operator()(const boost::shared_ptr<const M> &msg)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
    }
  };

public:
  void msg_gen()
  {
    ros::WallRate rate(100); // publish messages @ 100Hz
    const std::string frame_id("target");
    while (ros::ok() && run)
    {
      source.generate(frame_id, ros::Time::now());
      rate.sleep();
    }
  };

  void frame_gen()
  {
    ros::WallRate rate(50); // publish frame info @ 50 Hz (slower than msgs)
    while (ros::ok() && run)
    {
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = "base";
      transform.child_frame_id = "target";
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;
      buffer.setTransform(transform, "frame_generator", false);
      rate.sleep();
    }
  };

  void add_filter(int i, ros::CallbackQueueInterface *queue)
  {
    std::string name(queue ? "Q" : "S");
    name += std::to_string(i);

    filters.emplace_back(buffer, "base", i + 1, queue);
    auto &f = filters.back();
    f.setName(name);
    f.connectInput(source);
    f.registerCallback(Sink(name, 1));
  };
};

TEST_P(MessageFilterFixture, StressTest)
{
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::thread msg_gen(&MessageFilterFixture::msg_gen, this);
  std::thread frame_gen(&MessageFilterFixture::frame_gen, this);

  bool use_cbqueue = GetParam();
  ros::CallbackQueueInterface *queue = use_cbqueue ? nh.getCallbackQueue() : nullptr;
  // use fewer filters for signal-only transmission as we can remove only a single filter per iteration
  int num_filters = use_cbqueue ? 50 : 10;
  for (int i = 0; i < num_filters; ++i)
    add_filter(i, queue);

  // slowly remove filters
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  while (!filters.empty())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(7));
    filters.pop_front();
  }

  run = false;
  msg_gen.join();
  frame_gen.join();
}
INSTANTIATE_TEST_CASE_P(MessageFilterTests, MessageFilterFixture, ::testing::Values(false, true));

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tf2_ros_message_filter");
  return RUN_ALL_TESTS();
}
