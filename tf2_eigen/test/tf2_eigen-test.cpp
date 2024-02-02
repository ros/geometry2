/*
 * Copyright (c) Koji Terada
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

// To get M_PI, especially on Windows.

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <math.h>


#include <gtest/gtest.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


#include <memory>

TEST(TfEigen, ConvertVector3dStamped)
{
  const tf2::Stamped<Eigen::Vector3d> v(Eigen::Vector3d(1,2,3), ros::Time(5), "test");

  tf2::Stamped<Eigen::Vector3d> v1;
  geometry_msgs::PointStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v, v1);
}

TEST(TfEigen, ConvertVector3d)
{
  const Eigen::Vector3d v(1,2,3);

  Eigen::Vector3d v1;
  geometry_msgs::Point p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v, v1);
}

TEST(TfEigen, ConvertQuaterniondStamped)
{
  const tf2::Stamped<Eigen::Quaterniond> v(Eigen::Quaterniond(1,2,3,4), ros::Time(5), "test");

  tf2::Stamped<Eigen::Quaterniond> v1;
  geometry_msgs::QuaternionStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
  EXPECT_EQ(v.w(), v1.w());
  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfEigen, ConvertQuaterniond)
{
  const Eigen::Quaterniond v(1,2,3,4);

  Eigen::Quaterniond v1;
  geometry_msgs::Quaternion p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.w(), v1.w());
  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfEigen, ConvertAffine3dStamped)
{
  const Eigen::Affine3d v_nonstamped(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));
  const tf2::Stamped<Eigen::Affine3d> v(v_nonstamped, ros::Time(42), "test_frame");

  tf2::Stamped<Eigen::Affine3d> v1;
  geometry_msgs::PoseStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
}

TEST(TfEigen, ConvertIsometry3dStamped)
{
  const Eigen::Isometry3d v_nonstamped(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));
  const tf2::Stamped<Eigen::Isometry3d> v(v_nonstamped, ros::Time(42), "test_frame");

  tf2::Stamped<Eigen::Isometry3d> v1;
  geometry_msgs::PoseStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
}

TEST(TfEigen, ConvertAffine3d)
{
  const Eigen::Affine3d v(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));

  Eigen::Affine3d v1;
  geometry_msgs::Pose p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
}

TEST(TfEigen, ConvertIsometry3d)
{
  const Eigen::Isometry3d v(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d v1;
  geometry_msgs::Pose p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
}

TEST(TfEigen, ConvertTransform)
{
  Eigen::Matrix4d tm;

  double alpha = M_PI/4.0;
  double theta = M_PI/6.0;
  double gamma = M_PI/12.0;

  tm << cos(theta)*cos(gamma),-cos(theta)*sin(gamma),sin(theta), 1, //
  cos(alpha)*sin(gamma)+sin(alpha)*sin(theta)*cos(gamma),cos(alpha)*cos(gamma)-sin(alpha)*sin(theta)*sin(gamma),-sin(alpha)*cos(theta), 2, //
  sin(alpha)*sin(gamma)-cos(alpha)*sin(theta)*cos(gamma),cos(alpha)*sin(theta)*sin(gamma)+sin(alpha)*cos(gamma),cos(alpha)*cos(theta), 3, //
  0, 0, 0, 1;

  Eigen::Affine3d T(tm);

  geometry_msgs::TransformStamped msg = tf2::eigenToTransform(T);
  Eigen::Affine3d Tback = tf2::transformToEigen(msg);

  EXPECT_TRUE(T.isApprox(Tback));
  EXPECT_TRUE(tm.isApprox(Tback.matrix()));

  // same for Isometry
  Eigen::Isometry3d I(tm);

  msg = tf2::eigenToTransform(T);
  Eigen::Isometry3d Iback = tf2::transformToEigen(msg);

  EXPECT_TRUE(I.isApprox(Iback));
  EXPECT_TRUE(tm.isApprox(Iback.matrix()));
}

struct EigenTransform : public ::testing::Test
{
  static void SetUpTestSuite()
  {
    geometry_msgs::TransformStamped t;
    transform.transform.translation.x = 10;
    transform.transform.translation.y = 20;
    transform.transform.translation.z = 30;
    transform.transform.rotation.w = 0;
    transform.transform.rotation.x = 1;
    transform.transform.rotation.y = 0;
    transform.transform.rotation.z = 0;
    transform.header.stamp = ros::Time(2.0);
    transform.header.frame_id = "A";
    transform.child_frame_id = "B";
  }

  template<int mode>
  void testEigenTransform();

  ::testing::AssertionResult doTestEigenQuaternion(
    const Eigen::Quaterniond & parameter, const Eigen::Quaterniond & expected);

  static geometry_msgs::TransformStamped transform;
  static constexpr double EPS = 1e-3;
};

geometry_msgs::TransformStamped EigenTransform::transform;

template<int mode>
void EigenTransform::testEigenTransform()
{
  using T = Eigen::Transform<double, 3, mode>;
  using stampedT = tf2::Stamped<T>;

  const stampedT i1{
    T{Eigen::Translation3d{1, 2, 3} *Eigen::Quaterniond{0, 1, 0, 0}}, ros::Time(2), "A"};

  stampedT i_simple;
  tf2::doTransform(i1, i_simple, transform);

  EXPECT_NEAR(i_simple.translation().x(), 11, EPS);
  EXPECT_NEAR(i_simple.translation().y(), 18, EPS);
  EXPECT_NEAR(i_simple.translation().z(), 27, EPS);
  const auto q1 = Eigen::Quaterniond(i_simple.linear());
  EXPECT_NEAR(q1.x(), 0.0, EPS);
  EXPECT_NEAR(q1.y(), 0.0, EPS);
  EXPECT_NEAR(q1.z(), 0.0, EPS);
  EXPECT_NEAR(q1.w(), 1.0, EPS);
}

TEST_F(EigenTransform, Affine3d) {
  testEigenTransform<Eigen::Affine>();
}

TEST_F(EigenTransform, Isometry3d) {
  testEigenTransform<Eigen::Isometry>();
}

TEST_F(EigenTransform, Vector)
{
  const tf2::Stamped<Eigen::Vector3d> v1{{1, 2, 3}, ros::Time(2), "A"};

  // simple api
  tf2::Stamped<Eigen::Vector3d> v_simple;
  tf2::doTransform(v1, v_simple, transform);

  EXPECT_NEAR(v_simple.x(), 11, EPS);
  EXPECT_NEAR(v_simple.y(), 18, EPS);
  EXPECT_NEAR(v_simple.z(), 27, EPS);
}

// helper method for Quaternion tests
::testing::AssertionResult EigenTransform::doTestEigenQuaternion(
  const Eigen::Quaterniond & parameter, const Eigen::Quaterniond & expected)
{
  const tf2::Stamped<Eigen::Quaterniond> q1{parameter, ros::Time(2), "A"};
  // avoid linking error
  const double eps = EPS;

  // simple api
  tf2::Stamped<Eigen::Quaterniond> q_simple;
  tf2::doTransform(q1, q_simple, transform);
  // compare rotation matrices, as the quaternions can be ambigous
  EXPECT_TRUE(q_simple.toRotationMatrix().isApprox(expected.toRotationMatrix(), eps));

  return ::testing::AssertionSuccess();
}

TEST_F(EigenTransform, QuaternionRotY)
{
  // rotated by -90° around y
  // 0, 0, -1
  // 0, 1, 0,
  // 1, 0, 0
  const Eigen::Quaterniond param{Eigen::AngleAxisd(-1 * M_PI_2, Eigen::Vector3d::UnitY())};
  const Eigen::Quaterniond expected{0, M_SQRT1_2, 0, -1 * M_SQRT1_2};
  EXPECT_TRUE(doTestEigenQuaternion(param, expected));
}

TEST_F(EigenTransform, QuaternionRotX)
{
  // rotated by 90° around y
  const Eigen::Quaterniond param{Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX())};
  const Eigen::Quaterniond expected{Eigen::AngleAxisd(-1 * M_PI_2, Eigen::Vector3d::UnitX())};
  EXPECT_TRUE(doTestEigenQuaternion(param, expected));
}

TEST_F(EigenTransform, QuaternionRotZ)
{
  // rotated by 180° around z
  const Eigen::Quaterniond param{Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())};
  const Eigen::Quaterniond expected{Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())};
  EXPECT_TRUE(doTestEigenQuaternion(param, expected));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
