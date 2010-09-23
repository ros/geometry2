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


#include <tf2_kdl/tf2_kdl.h>
#include <ros/ros.h>
#include <kdl/frames_io.hpp>
#include <gtest/gtest.h>


tf2::Buffer tf_buffer;


TEST(TfDKL, Vector)
{
  tf2::Stamped<KDL::Vector> v1(KDL::Vector(1,2,3), ros::Time(2.0), "A");


  // simple api
  KDL::Vector v_simple = tf_buffer.transform(v1, "B", ros::Duration(2.0));
  EXPECT_EQ(v_simple[0], -9);
  EXPECT_EQ(v_simple[1], 18);
  EXPECT_EQ(v_simple[2], 27);

  // advanced api
  KDL::Vector v_advanced = tf_buffer.transform(v1, "B", ros::Time(2.0),
					       "A", ros::Duration(3.0));
  EXPECT_EQ(v_simple[0], -9);
  EXPECT_EQ(v_simple[1], 18);
  EXPECT_EQ(v_simple[2], 27);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  // populate buffer
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.x = 1;
  t.header.stamp = ros::Time(2.0);
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  tf_buffer.setTransform(t, "test");

  return RUN_ALL_TESTS();
}
