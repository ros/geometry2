/*
 * Copyright (c) 2020, Andrey Stepanov
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

#include <tf2_urdf/tf2_urdf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <urdf_model/pose.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <cmath>

double rand_double(const double max_value) {
    return static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) * max_value;
}

TEST(TfURDF, TFVectorToURDFVector) {
    const tf2::Vector3 tf_vector(rand_double(10.), rand_double(10.), rand_double(10.));
    urdf::Vector3 urdf_vector;
    tf2::convert(tf_vector, urdf_vector);
    ASSERT_DOUBLE_EQ(tf_vector.x(), urdf_vector.x);
    ASSERT_DOUBLE_EQ(tf_vector.y(), urdf_vector.y);
    ASSERT_DOUBLE_EQ(tf_vector.z(), urdf_vector.z);
}

TEST(TfURDF, URDFVectorToTFVector) {
    const urdf::Vector3 urdf_vector(rand_double(10.), rand_double(10.), rand_double(10.));
    tf2::Vector3 tf_vector;
    tf2::convert(urdf_vector, tf_vector);
    ASSERT_DOUBLE_EQ(urdf_vector.x, tf_vector.x());
    ASSERT_DOUBLE_EQ(urdf_vector.y, tf_vector.y());
    ASSERT_DOUBLE_EQ(urdf_vector.z, tf_vector.z());
}

TEST(TfURDF, Point32ToURDFVector) {
    geometry_msgs::Point32 point;
    point.x = rand_double(10.);
    point.y = rand_double(10.);
    point.z = rand_double(10.);
    urdf::Vector3 urdf_vector;
    tf2::convert(point, urdf_vector);
    ASSERT_DOUBLE_EQ(point.x, urdf_vector.x);
    ASSERT_DOUBLE_EQ(point.y, urdf_vector.y);
    ASSERT_DOUBLE_EQ(point.z, urdf_vector.z);
}

TEST(TfURDF, URDFVectorToPoint32) {
    const urdf::Vector3 urdf_vector(rand_double(10.), rand_double(10.), rand_double(10.));
    geometry_msgs::Point32 point;
    tf2::toMsg(urdf_vector, point);
    ASSERT_FLOAT_EQ(urdf_vector.x, point.x);
    ASSERT_FLOAT_EQ(urdf_vector.y, point.y);
    ASSERT_FLOAT_EQ(urdf_vector.z, point.z);
}

TEST(TfURDF, TFQuaternionToURDFRotation) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(rand_double(M_PI), rand_double(M_PI), rand_double(M_PI));
    urdf::Rotation urdf_quat;
    tf2::convert(tf_quat, urdf_quat);
    ASSERT_DOUBLE_EQ(tf_quat.x(), urdf_quat.x);
    ASSERT_DOUBLE_EQ(tf_quat.y(), urdf_quat.y);
    ASSERT_DOUBLE_EQ(tf_quat.z(), urdf_quat.z);
    ASSERT_DOUBLE_EQ(tf_quat.w(), urdf_quat.w);
}

TEST(TfURDF, URDFRotationToTFQuaternion) {
    urdf::Rotation urdf_quat;
    urdf_quat.setFromRPY(rand_double(M_PI), rand_double(M_PI), rand_double(M_PI));
    tf2::Quaternion tf_quat;
    tf2::convert(urdf_quat, tf_quat);
    ASSERT_DOUBLE_EQ(urdf_quat.x, tf_quat.x());
    ASSERT_DOUBLE_EQ(urdf_quat.y, tf_quat.y());
    ASSERT_DOUBLE_EQ(urdf_quat.z, tf_quat.z());
    ASSERT_DOUBLE_EQ(urdf_quat.w, tf_quat.w());
}

TEST(TfURDF, PoseToURDFPose) {
    geometry_msgs::Pose pose;
    pose.position.x = rand_double(10.);
    pose.position.y = rand_double(10.);
    pose.position.z = rand_double(10.);
    {
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(rand_double(M_PI), rand_double(M_PI), rand_double(M_PI));
        tf2::convert(tf_quat, pose.orientation);
    }
    urdf::Pose urdf_pose;
    tf2::convert(pose, urdf_pose);
    ASSERT_DOUBLE_EQ(pose.position.x, urdf_pose.position.x);
    ASSERT_DOUBLE_EQ(pose.position.y, urdf_pose.position.y);
    ASSERT_DOUBLE_EQ(pose.position.z, urdf_pose.position.z);
    ASSERT_DOUBLE_EQ(pose.orientation.x, urdf_pose.rotation.x);
    ASSERT_DOUBLE_EQ(pose.orientation.y, urdf_pose.rotation.y);
    ASSERT_DOUBLE_EQ(pose.orientation.z, urdf_pose.rotation.z);
    ASSERT_DOUBLE_EQ(pose.orientation.w, urdf_pose.rotation.w);
}

TEST(TfURDF, URDFPoseToPose) {
    urdf::Pose urdf_pose;
    urdf_pose.position.x = rand_double(10.);
    urdf_pose.position.y = rand_double(10.);
    urdf_pose.position.z = rand_double(10.);
    urdf_pose.rotation.setFromRPY(rand_double(M_PI), rand_double(M_PI), rand_double(M_PI));
    geometry_msgs::Pose pose;
    tf2::convert(urdf_pose, pose);
    ASSERT_DOUBLE_EQ(urdf_pose.position.x, pose.position.x);
    ASSERT_DOUBLE_EQ(urdf_pose.position.y, pose.position.y);
    ASSERT_DOUBLE_EQ(urdf_pose.position.z, pose.position.z);
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.x, pose.orientation.x);
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.y, pose.orientation.y);
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.z, pose.orientation.z);
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.w, pose.orientation.w);
}

TEST(TfURDF, TFTransformToURDFPose) {
    tf2::Transform tf_transform;
    tf_transform.setOrigin(tf2::Vector3(rand_double(10.), rand_double(10.), rand_double(10.)));
    {
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(rand_double(M_PI), rand_double(M_PI), rand_double(M_PI));
        tf_transform.setRotation(tf_quat);
    }
    urdf::Pose urdf_pose;
    tf2::convert(tf_transform, urdf_pose);
    const tf2::Vector3& translation = tf_transform.getOrigin();
    ASSERT_DOUBLE_EQ(translation.x(), urdf_pose.position.x);
    ASSERT_DOUBLE_EQ(translation.y(), urdf_pose.position.y);
    ASSERT_DOUBLE_EQ(translation.z(), urdf_pose.position.z);
    tf2::Quaternion tf_quat;
    tf_transform.getBasis().getRotation(tf_quat);
    ASSERT_DOUBLE_EQ(tf_quat.x(), urdf_pose.rotation.x);
    ASSERT_DOUBLE_EQ(tf_quat.y(), urdf_pose.rotation.y);
    ASSERT_DOUBLE_EQ(tf_quat.z(), urdf_pose.rotation.z);
    ASSERT_DOUBLE_EQ(tf_quat.w(), urdf_pose.rotation.w);
}

TEST(TfURDF, URDFPoseToTFTransform) {
    urdf::Pose urdf_pose;
    urdf_pose.position.x = rand_double(10.);
    urdf_pose.position.y = rand_double(10.);
    urdf_pose.position.z = rand_double(10.);
    urdf_pose.rotation.setFromRPY(rand_double(M_PI), rand_double(M_PI), rand_double(M_PI));
    tf2::Transform tf_transform;
    tf2::convert(urdf_pose, tf_transform);
    const tf2::Vector3& translation = tf_transform.getOrigin();
    ASSERT_DOUBLE_EQ(urdf_pose.position.x, translation.x());
    ASSERT_DOUBLE_EQ(urdf_pose.position.y, translation.y());
    ASSERT_DOUBLE_EQ(urdf_pose.position.z, translation.z());
    tf2::Quaternion tf_quat;
    tf_transform.getBasis().getRotation(tf_quat);
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.x, tf_quat.x());
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.y, tf_quat.y());
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.z, tf_quat.z());
    ASSERT_DOUBLE_EQ(urdf_pose.rotation.w, tf_quat.w());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

