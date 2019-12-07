/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <gtest/gtest.h>
#include <tf2/convert.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_bullet/tf2_bullet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/time.h>

TEST(tf2Convert, kdlToBullet)
{
  double epsilon = 1e-9;

  tf2::Stamped<btVector3> b(btVector3(1,2,3), ros::Time(), "my_frame");

  tf2::Stamped<btVector3> b1 = b;
  tf2::Stamped<KDL::Vector> k1;
  tf2::convert(b1, k1);

  tf2::Stamped<btVector3> b2;
  tf2::convert(k1, b2);

  EXPECT_EQ(b.frame_id_, b2.frame_id_);
  EXPECT_NEAR(b.stamp_.toSec(), b2.stamp_.toSec(), epsilon);
  EXPECT_NEAR(b.x(), b2.x(), epsilon);
  EXPECT_NEAR(b.y(), b2.y(), epsilon);
  EXPECT_NEAR(b.z(), b2.z(), epsilon);


  EXPECT_EQ(b1.frame_id_, b2.frame_id_);
  EXPECT_NEAR(b1.stamp_.toSec(), b2.stamp_.toSec(), epsilon);
  EXPECT_NEAR(b1.x(), b2.x(), epsilon);
  EXPECT_NEAR(b1.y(), b2.y(), epsilon);
  EXPECT_NEAR(b1.z(), b2.z(), epsilon);
}

TEST(tf2Convert, kdlBulletROSConversions)
{
  double epsilon = 1e-9;

  tf2::Stamped<btVector3> b1(btVector3(1,2,3), ros::Time(), "my_frame"), b2, b3, b4;
  geometry_msgs::PointStamped r1, r2, r3;
  tf2::Stamped<KDL::Vector> k1, k2, k3;

  // Do bullet -> self -> bullet -> KDL -> self -> KDL -> ROS -> self -> ROS -> KDL -> bullet -> ROS -> bullet
  tf2::convert(b1, b1);
  tf2::convert(b1, b2);
  tf2::convert(b2, k1);
  tf2::convert(k1, k1);
  tf2::convert(k1, k2);
  tf2::convert(k2, r1);
  tf2::convert(r1, r1);
  tf2::convert(r1, r2);
  tf2::convert(r2, k3);
  tf2::convert(k3, b3);
  tf2::convert(b3, r3);
  tf2::convert(r3, b4);

  EXPECT_EQ(b1.frame_id_, b4.frame_id_);
  EXPECT_NEAR(b1.stamp_.toSec(), b4.stamp_.toSec(), epsilon);
  EXPECT_NEAR(b1.x(), b4.x(), epsilon);
  EXPECT_NEAR(b1.y(), b4.y(), epsilon);
  EXPECT_NEAR(b1.z(), b4.z(), epsilon);
}

TEST(tf2Convert, PoseStampedConversions) {
  double epsilon = 1e-9;

  const tf2::Stamped<tf2::Transform> p_tf2_1(tf2::Transform(tf2::Quaternion(1.0,0.0,0.0,2.0), tf2::Vector3(1,2,3)), ros::Time(), "my_frame");
  tf2::Stamped<tf2::Transform> p_tf2_2;
  geometry_msgs::PoseStamped msg;
  tf2::toMsg(p_tf2_1, msg);

  tf2::Stamped<Eigen::Isometry3d> p_e_iso;
  tf2::Stamped<Eigen::Affine3d> p_e_aff;
  tf2::Stamped<KDL::Frame> p_f;

  tf2::convert(p_tf2_1, p_e_iso);
  tf2::convert(p_e_iso, p_f);
  tf2::convert(p_f, p_e_aff);
  tf2::convert(p_e_aff, p_tf2_2);

  tf2::toMsg(p_e_aff, msg);

  EXPECT_EQ(p_tf2_1.frame_id_, p_tf2_2.frame_id_);
  EXPECT_NEAR(p_tf2_1.stamp_.toSec(), p_tf2_2.stamp_.toSec(), epsilon);

  const auto& q1(p_tf2_1.getRotation()), q2(p_tf2_2.getRotation());
  EXPECT_NEAR(q1.x(), q2.x(), epsilon);
  EXPECT_NEAR(q1.y(), q2.y(), epsilon);
  EXPECT_NEAR(q1.z(), q2.z(), epsilon);
  EXPECT_NEAR(q1.w(), q2.w(), epsilon);

  const auto& o1(p_tf2_1.getOrigin()), o2(p_tf2_2.getOrigin());
  EXPECT_NEAR(o1.x(), o2.x(), epsilon);
  EXPECT_NEAR(o1.y(), o2.y(), epsilon);
  EXPECT_NEAR(o1.z(), o2.z(), epsilon);
}

TEST(tf2Convert, QuaternionStampedConversations)
{
  const double epsilon = 1e-9;
  const tf2::Stamped<Eigen::Quaterniond> q_e_1(Eigen::Quaterniond(2.0, 4.0, 0.25, -1),
                                               ros::Time(), "my_frame");
  tf2::Stamped<tf2::Quaternion> q_tf_1;
  tf2::convert(q_e_1, q_tf_1);

  EXPECT_EQ(q_e_1.frame_id_, q_tf_1.frame_id_);
  EXPECT_NEAR(q_e_1.stamp_.toSec(), q_tf_1.stamp_.toSec(), epsilon);
  EXPECT_NEAR(q_e_1.x(), q_tf_1.x(), epsilon);
  EXPECT_NEAR(q_e_1.y(), q_tf_1.y(), epsilon);
  EXPECT_NEAR(q_e_1.z(), q_tf_1.z(), epsilon);
  EXPECT_NEAR(q_e_1.w(), q_tf_1.w(), epsilon);

  tf2::Stamped<Eigen::Quaterniond> q_e_2;
  tf2::convert(q_tf_1, q_e_2);

  EXPECT_EQ(q_e_2.frame_id_, q_tf_1.frame_id_);
  EXPECT_NEAR(q_e_2.stamp_.toSec(), q_tf_1.stamp_.toSec(), epsilon);
  EXPECT_NEAR(q_e_2.x(), q_tf_1.x(), epsilon);
  EXPECT_NEAR(q_e_2.y(), q_tf_1.y(), epsilon);
  EXPECT_NEAR(q_e_2.z(), q_tf_1.z(), epsilon);
  EXPECT_NEAR(q_e_2.w(), q_tf_1.w(), epsilon);
}

TEST(tf2Convert, QuaternionConversations)
{
  const double epsilon = 1e-9;
  const Eigen::Quaterniond q_e_1(2.0, 4.0, 0.25, -1);
  tf2::Quaternion q_tf_1;
  tf2::convert(q_e_1, q_tf_1);

  EXPECT_NEAR(q_e_1.x(), q_tf_1.x(), epsilon);
  EXPECT_NEAR(q_e_1.y(), q_tf_1.y(), epsilon);
  EXPECT_NEAR(q_e_1.z(), q_tf_1.z(), epsilon);
  EXPECT_NEAR(q_e_1.w(), q_tf_1.w(), epsilon);

  Eigen::Quaterniond q_e_2;
  tf2::convert(q_tf_1, q_e_2);

  EXPECT_NEAR(q_e_2.x(), q_tf_1.x(), epsilon);
  EXPECT_NEAR(q_e_2.y(), q_tf_1.y(), epsilon);
  EXPECT_NEAR(q_e_2.z(), q_tf_1.z(), epsilon);
  EXPECT_NEAR(q_e_2.w(), q_tf_1.w(), epsilon);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

