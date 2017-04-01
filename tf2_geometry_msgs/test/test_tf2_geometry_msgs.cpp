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


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::Buffer* tf_buffer;
static const double EPS = 1e-3;


TEST(TfGeometry, Frame)
{
  geometry_msgs::PoseStamped v1;
  v1.pose.position.x = 1;
  v1.pose.position.y = 2;
  v1.pose.position.z = 3;
  v1.pose.orientation.x = 1;
  v1.header.stamp = ros::Time(2);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::PoseStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_simple.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_simple.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.w, 1.0, EPS);
  

  // advanced api
  geometry_msgs::PoseStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
							      "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_advanced.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_advanced.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.w, 1.0, EPS);
}

void expect_vector3_same(const geometry_msgs::Vector3 & m1,
    const geometry_msgs::Vector3 & m2)
{
  EXPECT_NEAR(m1.x, m2.x, EPS);
  EXPECT_NEAR(m1.y, m2.y, EPS);
  EXPECT_NEAR(m1.z, m2.z, EPS);
}

void expect_vector3_same(const tf2::Vector3 & tf1,
    const geometry_msgs::Vector3 & m2)
{
  EXPECT_NEAR(tf1.x(), m2.x, EPS);
  EXPECT_NEAR(tf1.y(), m2.y, EPS);
  EXPECT_NEAR(tf1.z(), m2.z, EPS);
}

void expect_vector3_same(const tf2::Vector3 & tf1,
    const tf2::Vector3 & tf2)
{
  EXPECT_NEAR(tf1.x(), tf2.x(), EPS);
  EXPECT_NEAR(tf1.y(), tf2.y(), EPS);
  EXPECT_NEAR(tf1.z(), tf2.z(), EPS);
}

void expect_vector3stamped_same(const geometry_msgs::Vector3Stamped & m1,
                                const geometry_msgs::Vector3Stamped & m2)
{
  expect_vector3_same(m1.vector, m2.vector);
  EXPECT_EQ(m1.header.stamp, m2.header.stamp);
  EXPECT_EQ(m1.header.frame_id, m2.header.frame_id);
}

void expect_vector3stamped_same(const tf2::Stamped<tf2::Vector3> & tf1,
                                const geometry_msgs::Vector3Stamped & m2)
{
  expect_vector3_same((tf2::Vector3)tf1, m2.vector);
  EXPECT_EQ(tf1.stamp_, m2.header.stamp);
  EXPECT_EQ(tf1.frame_id_, m2.header.frame_id);
}

void expect_vector3stamped_same(const tf2::Stamped<tf2::Vector3> & tf1,
                                const tf2::Stamped<tf2::Vector3> & tf2)
{
  expect_vector3_same((tf2::Vector3)tf1, (tf2::Vector3)tf2);
  EXPECT_EQ(tf1.stamp_, tf2.stamp_);
  EXPECT_EQ(tf1.frame_id_, tf2.frame_id_);
}

TEST(TfGeometry, Vector)
{
  geometry_msgs::Vector3Stamped v1, res;
  v1.vector.x = 1;
  v1.vector.y = 2;
  v1.vector.z = 3;
  v1.header.stamp = ros::Time(2.0);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::Vector3Stamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.vector.x, 1, EPS);
  EXPECT_NEAR(v_simple.vector.y, -2, EPS);
  EXPECT_NEAR(v_simple.vector.z, -3, EPS);

  // advanced api
  geometry_msgs::Vector3Stamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
								 "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.vector.x, 1, EPS);
  EXPECT_NEAR(v_advanced.vector.y, -2, EPS);
  EXPECT_NEAR(v_advanced.vector.z, -3, EPS);
}

TEST(Tf2GeometryMsgs, Vector3Conversions)
{
  geometry_msgs::Vector3Stamped original, mres, mres_self;
  original.vector.x = 1;
  original.vector.y = 2;
  original.vector.z = 3;
  original.header.stamp = ros::Time(2.0);
  original.header.frame_id = "A";

  tf2::Stamped<tf2::Vector3> out, out_self;

  // Test fromMsg
  tf2:convert(original, out);
  expect_vector3stamped_same(out, original);

  // Test identity conversion as stamped
  tf2::convert(out, out_self);
  expect_vector3stamped_same(out, out_self);
  
  // Test identity conversion as message
  tf2::convert(original, mres_self);
  expect_vector3stamped_same(original, mres_self);
  
  geometry_msgs::Vector3 simple, pres2, pres3;
  tf2::Vector3 out2;
  simple = original.vector;
  // Test fromMsg non-stamped
  tf2::convert(simple, out2);
  // expect_vector3_same(out, simple);

  // Test toMsg
  //TODO not working tf2::convert<tf2::Stamped<tf2::Vector3>, geometry_msgs::Vector3Stamped>(out, mres);
  // expect_vector3stamped_same(mres, original);
  
  // Test toMsg non stamped
  tf2::convert(out2, pres2);
  expect_vector3_same((tf2::Vector3)out, pres2);
}

void expect_point_same(const geometry_msgs::Point & m1,
    const geometry_msgs::Point & m2)
{
  EXPECT_NEAR(m1.x, m2.x, EPS);
  EXPECT_NEAR(m1.y, m2.y, EPS);
  EXPECT_NEAR(m1.z, m2.z, EPS);
}

void expect_point_same(const tf2::Vector3 & tf1,
    const geometry_msgs::Point & m2)
{
  EXPECT_NEAR(tf1.x(), m2.x, EPS);
  EXPECT_NEAR(tf1.y(), m2.y, EPS);
  EXPECT_NEAR(tf1.z(), m2.z, EPS);
}

void expect_point_same(const tf2::Vector3 & tf1,
    const tf2::Vector3 & tf2)
{
  EXPECT_NEAR(tf1.x(), tf2.x(), EPS);
  EXPECT_NEAR(tf1.y(), tf2.y(), EPS);
  EXPECT_NEAR(tf1.z(), tf2.z(), EPS);
}

void expect_pointstamped_same(const geometry_msgs::PointStamped & m1,
                                const geometry_msgs::PointStamped & m2)
{
  expect_point_same(m1.point, m2.point);
  EXPECT_EQ(m1.header.stamp, m2.header.stamp);
  EXPECT_EQ(m1.header.frame_id, m2.header.frame_id);
}

void expect_pointstamped_same(const tf2::Stamped<tf2::Vector3> & tf1,
                                const geometry_msgs::PointStamped & m2)
{
  expect_point_same((tf2::Vector3)tf1, m2.point);
  EXPECT_EQ(tf1.stamp_, m2.header.stamp);
  EXPECT_EQ(tf1.frame_id_, m2.header.frame_id);
}

void expect_pointstamped_same(const tf2::Stamped<tf2::Vector3> & tf1,
                                const tf2::Stamped<tf2::Vector3> & tf2)
{
  expect_point_same((tf2::Vector3)tf1, (tf2::Vector3)tf2);
  EXPECT_EQ(tf1.stamp_, tf2.stamp_);
  EXPECT_EQ(tf1.frame_id_, tf2.frame_id_);
}

TEST(TfGeometry, Point)
{
  geometry_msgs::PointStamped v1, res;
  v1.point.x = 1;
  v1.point.y = 2;
  v1.point.z = 3;
  v1.header.stamp = ros::Time(2.0);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::PointStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.point.x, -9, EPS);
  EXPECT_NEAR(v_simple.point.y, 18, EPS);
  EXPECT_NEAR(v_simple.point.z, 27, EPS);

  // advanced api
  geometry_msgs::PointStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
								 "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.point.x, -9, EPS);
  EXPECT_NEAR(v_advanced.point.y, 18, EPS);
  EXPECT_NEAR(v_advanced.point.z, 27, EPS);
}

TEST(Tf2GeometryMsgs, PointConversions)
{
  geometry_msgs::PointStamped original, mres, mres_self;
  original.point.x = 1;
  original.point.y = 2;
  original.point.z = 3;
  original.header.stamp = ros::Time(2.0);
  original.header.frame_id = "A";

  tf2::Stamped<tf2::Vector3> out, out_self;

  // Test fromMsg
  tf2:convert(original, out);
  expect_pointstamped_same(out, original);

  // Test identity conversion as stamped
  tf2::convert(out, out_self);
  expect_pointstamped_same(out, out_self);
  
  // Test identity conversion as message
  tf2::convert(original, mres_self);
  expect_pointstamped_same(original, mres_self);
  
  geometry_msgs::Point simple, pres2, pres3;
  tf2::Vector3 out2;
  simple = original.point;
  // Test fromMsg non-stamped
  tf2::convert(simple, out2);
  expect_point_same(out, simple);

  // TODO NOT working // Test toMsg
  // tf2::convert(out, mres);
  // expect_pointstamped_same(mres, original);
  
  // Test toMsg non stamped
  tf2::convert(out2, pres2);
  expect_point_same((tf2::Vector3)out, pres2);
}

TEST(Tf2GeometryMsgs, QuaternionConversions)
{
  geometry_msgs::QuaternionStamped m1, mres, mres_self;
  m1.quaternion.x = 0;
  m1.quaternion.y = 0;
  m1.quaternion.z = 1;
  m1.quaternion.w = 0;
  m1.header.stamp = ros::Time(2.0);
  m1.header.frame_id = "A";

  // Test fromMsg
  tf2::Stamped<tf2::Quaternion> out, out_self;
  tf2:convert(m1, out);
  EXPECT_NEAR(out.getX(), 0, EPS);
  EXPECT_NEAR(out.getY(), 0, EPS);
  EXPECT_NEAR(out.getZ(), 1, EPS);
  EXPECT_NEAR(out.w(), 0, EPS);
  EXPECT_EQ(out.stamp_, m1.header.stamp);
  EXPECT_EQ(out.frame_id_, m1.header.frame_id);

  // Test identity conversion as stamped
  tf2::convert(out, out_self);
  EXPECT_NEAR(out_self.getX(), 0, EPS);
  EXPECT_NEAR(out_self.getY(), 0, EPS);
  EXPECT_NEAR(out_self.getZ(), 1, EPS);
  EXPECT_NEAR(out_self.w(), 0, EPS);
  EXPECT_EQ(out_self.stamp_, m1.header.stamp);
  EXPECT_EQ(out_self.frame_id_, m1.header.frame_id);

  geometry_msgs::Quaternion p2, pres2, pres3;
  tf2::Quaternion out2;
  p2 = m1.quaternion;
  // Test fromMsg
  tf2::convert(p2, out2);
  EXPECT_NEAR(out2.getX(), 0, EPS);
  EXPECT_NEAR(out2.getY(), 0, EPS);
  EXPECT_NEAR(out2.getZ(), 1, EPS);
  EXPECT_NEAR(out2.w(), 0, EPS);

  // TODO // Test toMsg
  // tf2::convert<tf2::Stamped<tf2::Vector3>, geometry_msgs::QuaternionStamped>(out, mres);
  // EXPECT_NEAR(mres.quaternion.x, 1, EPS);
  // EXPECT_NEAR(mres.quaternion.y, 2, EPS);
  // EXPECT_NEAR(mres.quaternion.z, 3, EPS);
  // EXPECT_EQ(mres.header.stamp, m1.header.stamp);
  // EXPECT_EQ(mres.header.frame_id, m1.header.frame_id);
  
  // Test no-op convert as message
  tf2::convert(mres, mres_self);
  EXPECT_NEAR(mres_self.quaternion.x, mres.quaternion.x, EPS);
  EXPECT_NEAR(mres_self.quaternion.y, mres.quaternion.y, EPS);
  EXPECT_NEAR(mres_self.quaternion.z, mres.quaternion.z, EPS);
  EXPECT_NEAR(mres_self.quaternion.w, mres.quaternion.w, EPS);
  EXPECT_EQ(mres_self.header.stamp, mres.header.stamp);
  EXPECT_EQ(mres_self.header.frame_id, mres.header.frame_id);

  // Test toMsg
  tf2::convert(out, pres2);
  EXPECT_NEAR(pres2.x, 0, EPS);
  EXPECT_NEAR(pres2.y, 0, EPS);
  EXPECT_NEAR(pres2.z, 1, EPS);
  EXPECT_NEAR(pres2.w, 0, EPS);
}

TEST(Tf2GeometryMsgs, PoseConversions)
{
  geometry_msgs::PoseStamped m1, mres, mres_self;
  m1.pose.position.x = 1;
  m1.pose.position.y = 2;
  m1.pose.position.z = 3;
  m1.pose.orientation.x = 0;
  m1.pose.orientation.y = 0;
  m1.pose.orientation.z = 1;
  m1.pose.orientation.w = 0;
  m1.header.stamp = ros::Time(2.0);
  m1.header.frame_id = "A";

  // Test fromMsg
  tf2::Stamped<tf2::Transform> out, out_self;
  tf2:convert(m1, out);
  EXPECT_NEAR(out.getOrigin().getX(), 1, EPS);
  EXPECT_NEAR(out.getOrigin().getY(), 2, EPS);
  EXPECT_NEAR(out.getOrigin().getZ(), 3, EPS);
  EXPECT_NEAR(out.getRotation().x(), 0, EPS);
  EXPECT_NEAR(out.getRotation().y(), 0, EPS);
  EXPECT_NEAR(out.getRotation().z(), 1, EPS);
  EXPECT_NEAR(out.getRotation().w(), 0, EPS);
  EXPECT_EQ(out.stamp_, m1.header.stamp);
  EXPECT_EQ(out.frame_id_, m1.header.frame_id);

  // Test identity conversion as stamped
  tf2::convert(out, out_self);
  EXPECT_NEAR(out_self.getOrigin().getX(), 1, EPS);
  EXPECT_NEAR(out_self.getOrigin().getY(), 2, EPS);
  EXPECT_NEAR(out_self.getOrigin().getZ(), 3, EPS);
  EXPECT_NEAR(out_self.getRotation().x(), 0, EPS);
  EXPECT_NEAR(out_self.getRotation().y(), 0, EPS);
  EXPECT_NEAR(out_self.getRotation().z(), 1, EPS);
  EXPECT_NEAR(out_self.getRotation().w(), 0, EPS);
  EXPECT_EQ(out_self.stamp_, m1.header.stamp);
  EXPECT_EQ(out_self.frame_id_, m1.header.frame_id);

  geometry_msgs::Pose p2, pres2, pres3;
  tf2::Transform out2;
  p2 = m1.pose;
  // Test fromMsg
  tf2::convert(p2, out2);
  EXPECT_NEAR(out2.getOrigin().getX(), 1, EPS);
  EXPECT_NEAR(out2.getOrigin().getY(), 2, EPS);
  EXPECT_NEAR(out2.getOrigin().getZ(), 3, EPS);
  EXPECT_NEAR(out2.getRotation().x(), 0, EPS);
  EXPECT_NEAR(out2.getRotation().y(), 0, EPS);
  EXPECT_NEAR(out2.getRotation().z(), 1, EPS);
  EXPECT_NEAR(out2.getRotation().w(), 0, EPS);

  // TODO // Test toMsg
  // tf2::convert<tf2::Stamped<tf2::Vector3>, geometry_msgs::PoseStamped>(out, mres);
  // EXPECT_NEAR(mres.quaternion.x, 1, EPS);
  // EXPECT_NEAR(mres.quaternion.y, 2, EPS);
  // EXPECT_NEAR(mres.quaternion.z, 3, EPS);
  // EXPECT_EQ(mres.header.stamp, m1.header.stamp);
  // EXPECT_EQ(mres.header.frame_id, m1.header.frame_id);
  
  // Test no-op convert as message
  tf2::convert(mres, mres_self);
  EXPECT_NEAR(mres_self.pose.position.x, mres.pose.position.x, EPS);
  EXPECT_NEAR(mres_self.pose.position.y, mres.pose.position.y, EPS);
  EXPECT_NEAR(mres_self.pose.position.z, mres.pose.position.z, EPS);
  EXPECT_NEAR(mres_self.pose.orientation.x, mres.pose.orientation.x, EPS);
  EXPECT_NEAR(mres_self.pose.orientation.y, mres.pose.orientation.y, EPS);
  EXPECT_NEAR(mres_self.pose.orientation.z, mres.pose.orientation.z, EPS);
  EXPECT_NEAR(mres_self.pose.orientation.w, mres.pose.orientation.w, EPS);
  EXPECT_EQ(mres_self.header.stamp, mres.header.stamp);
  EXPECT_EQ(mres_self.header.frame_id, mres.header.frame_id);
  
  // TODO // Test toMsg
  // tf2::convert(out2, pres2);
  // EXPECT_NEAR(pres2.position.x, 1, EPS);
  // EXPECT_NEAR(pres2.position.y, 2, EPS);
  // EXPECT_NEAR(pres2.position.z, 3, EPS);
  // EXPECT_NEAR(pres2.orientation.x, 0, EPS);
  // EXPECT_NEAR(pres2.orientation.y, 0, EPS);
  // EXPECT_NEAR(pres2.orientation.z, 1, EPS);
  // EXPECT_NEAR(pres2.orientation.w, 0, EPS);
}

TEST(Tf2GeometryMsgs, TransformConversions)
{
  geometry_msgs::TransformStamped m1, mres, mres_self;
  m1.transform.translation.x = 1;
  m1.transform.translation.y = 2;
  m1.transform.translation.z = 3;
  m1.transform.rotation.x = 0;
  m1.transform.rotation.y = 0;
  m1.transform.rotation.z = 1;
  m1.transform.rotation.w = 0;
  m1.header.stamp = ros::Time(2.0);
  m1.header.frame_id = "A";

  // Test fromMsg
  tf2::Stamped<tf2::Transform> out, out_self;
  tf2:convert(m1, out);
  EXPECT_NEAR(out.getOrigin().getX(), 1, EPS);
  EXPECT_NEAR(out.getOrigin().getY(), 2, EPS);
  EXPECT_NEAR(out.getOrigin().getZ(), 3, EPS);
  EXPECT_NEAR(out.getRotation().x(), 0, EPS);
  EXPECT_NEAR(out.getRotation().y(), 0, EPS);
  EXPECT_NEAR(out.getRotation().z(), 1, EPS);
  EXPECT_NEAR(out.getRotation().w(), 0, EPS);
  EXPECT_EQ(out.stamp_, m1.header.stamp);
  EXPECT_EQ(out.frame_id_, m1.header.frame_id);

  // Test identity conversion as stamped
  tf2::convert(out, out_self);
  EXPECT_NEAR(out_self.getOrigin().getX(), 1, EPS);
  EXPECT_NEAR(out_self.getOrigin().getY(), 2, EPS);
  EXPECT_NEAR(out_self.getOrigin().getZ(), 3, EPS);
  EXPECT_NEAR(out_self.getRotation().x(), 0, EPS);
  EXPECT_NEAR(out_self.getRotation().y(), 0, EPS);
  EXPECT_NEAR(out_self.getRotation().z(), 1, EPS);
  EXPECT_NEAR(out_self.getRotation().w(), 0, EPS);
  EXPECT_EQ(out_self.stamp_, m1.header.stamp);
  EXPECT_EQ(out_self.frame_id_, m1.header.frame_id);

  geometry_msgs::Transform p2, pres2, pres3;
  tf2::Transform out2;
  p2 = m1.transform;
  // Test fromMsg
  tf2::convert(p2, out2);
  EXPECT_NEAR(out2.getOrigin().getX(), 1, EPS);
  EXPECT_NEAR(out2.getOrigin().getY(), 2, EPS);
  EXPECT_NEAR(out2.getOrigin().getZ(), 3, EPS);
  EXPECT_NEAR(out2.getRotation().x(), 0, EPS);
  EXPECT_NEAR(out2.getRotation().y(), 0, EPS);
  EXPECT_NEAR(out2.getRotation().z(), 1, EPS);
  EXPECT_NEAR(out2.getRotation().w(), 0, EPS);

  // TODO // Test toMsg
  // tf2::convert<tf2::Stamped<tf2::Vector3>, geometry_msgs::TransformStamped>(out, mres);
  // EXPECT_NEAR(mres.quaternion.x, 1, EPS);
  // EXPECT_NEAR(mres.quaternion.y, 2, EPS);
  // EXPECT_NEAR(mres.quaternion.z, 3, EPS);
  // EXPECT_EQ(mres.header.stamp, m1.header.stamp);
  // EXPECT_EQ(mres.header.frame_id, m1.header.frame_id);
  
  // Test no-op convert as message
  tf2::convert(mres, mres_self);
  EXPECT_NEAR(mres_self.transform.translation.x, mres.transform.translation.x, EPS);
  EXPECT_NEAR(mres_self.transform.translation.y, mres.transform.translation.y, EPS);
  EXPECT_NEAR(mres_self.transform.translation.z, mres.transform.translation.z, EPS);
  EXPECT_NEAR(mres_self.transform.rotation.x, mres.transform.rotation.x, EPS);
  EXPECT_NEAR(mres_self.transform.rotation.y, mres.transform.rotation.y, EPS);
  EXPECT_NEAR(mres_self.transform.rotation.z, mres.transform.rotation.z, EPS);
  EXPECT_NEAR(mres_self.transform.rotation.w, mres.transform.rotation.w, EPS);
  EXPECT_EQ(mres_self.header.stamp, mres.header.stamp);
  EXPECT_EQ(mres_self.header.frame_id, mres.header.frame_id);
  
  // TODO // Test toMsg
  // tf2::convert(out2, pres2);
  // EXPECT_NEAR(pres2.translation.x, 1, EPS);
  // EXPECT_NEAR(pres2.translation.y, 2, EPS);
  // EXPECT_NEAR(pres2.translation.z, 3, EPS);
  // EXPECT_NEAR(pres2.rotation.x, 0, EPS);
  // EXPECT_NEAR(pres2.rotation.y, 0, EPS);
  // EXPECT_NEAR(pres2.rotation.z, 1, EPS);
  // EXPECT_NEAR(pres2.rotation.w, 0, EPS);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  tf_buffer = new tf2_ros::Buffer();

  // populate buffer
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.x = 1;
  t.header.stamp = ros::Time(2.0);
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  tf_buffer->setTransform(t, "test");

  bool ret = RUN_ALL_TESTS();
  delete tf_buffer;
  return ret;
}
