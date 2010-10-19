/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <tf2/buffer_core.h>

#include <ros/time.h>
#include <ros/assert.h>

#include <boost/lexical_cast.hpp>

int main(int argc, char** argv)
{
  uint32_t num_levels = 100;
  if (argc > 1)
  {
    num_levels = boost::lexical_cast<uint32_t>(argv[1]);
  }

  tf2::BufferCore bc;
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time(1);
  t.header.frame_id = "root";
  t.child_frame_id = "1";
  t.transform.rotation.w = 1.0;
  bc.setTransform(t, "me");

  for (uint32_t i = 2; i <= num_levels; ++i)
  {
    std::stringstream parent_ss;
    parent_ss << (i - 1);
    std::stringstream child_ss;
    child_ss << i;

    t.transform.translation.x = 1;
    t.header.frame_id = parent_ss.str();
    t.child_frame_id = child_ss.str();
    bc.setTransform(t, "me");
  }

  std::string levels_string = boost::lexical_cast<std::string>(num_levels);
  ros::WallTime start = ros::WallTime::now();
  for (int i = 0; i < 100000; ++i)
  {
    geometry_msgs::TransformStamped out_t = bc.lookupTransform("root", levels_string, ros::Time(0));
  }
  ros::WallTime end = ros::WallTime::now();
  ros::WallDuration dur = end - start;
  ROS_INFO("100000 %d-level transforms took %f for an average of %9f", num_levels, dur.toSec(), dur.toSec() / 100000.0);
  //ROS_INFO_STREAM(out_t);
}
