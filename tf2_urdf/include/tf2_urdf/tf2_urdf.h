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


#ifndef TF2_URDF_H_
#define TF2_URDF_H_

#include <urdf_model/pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2/convert.h>

namespace tf2 {

// ----------------------------------------------------------------------------

inline void fromMsg(const geometry_msgs::Vector3& in, urdf::Vector3& out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
}

inline geometry_msgs::Vector3 toMsg(const urdf::Vector3& in) {
    geometry_msgs::Vector3 msg;
    msg.x = in.x;
    msg.y = in.y;
    msg.z = in.z;
    return msg;
}

// ----------------------------------------------------------------------------

inline void fromMsg(const geometry_msgs::Point& in, urdf::Vector3& out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
}

inline geometry_msgs::Point toMsg(const urdf::Vector3& in, geometry_msgs::Point& out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
    return out;
}

// ----------------------------------------------------------------------------

inline void fromMsg(const geometry_msgs::Point32& in, urdf::Vector3& out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
}

inline geometry_msgs::Point32 toMsg(const urdf::Vector3& in, geometry_msgs::Point32& out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
    return out;
}

// ----------------------------------------------------------------------------

inline void fromMsg(const geometry_msgs::Quaternion& in, urdf::Rotation& out) {
    out.setFromQuaternion(in.x, in.y, in.z, in.w);
}

inline geometry_msgs::Quaternion toMsg(const urdf::Rotation& in) {
    geometry_msgs::Quaternion msg;
    msg.x = in.x;
    msg.y = in.y;
    msg.z = in.z;
    msg.w = in.w;
    return msg;
}

// ----------------------------------------------------------------------------

inline void fromMsg(const geometry_msgs::Pose& in, urdf::Pose& out) {
    fromMsg(in.position, out.position);
    fromMsg(in.orientation, out.rotation);
}

inline geometry_msgs::Pose toMsg(const urdf::Pose& in) {
    geometry_msgs::Pose msg;
    toMsg(in.position, msg.position);
    convert(in.rotation, msg.orientation);
    return msg;
}

// ----------------------------------------------------------------------------

inline void fromMsg(const geometry_msgs::Transform& in, urdf::Pose& out) {
    fromMsg(in.translation, out.position);
    fromMsg(in.rotation, out.rotation);
}

inline geometry_msgs::Transform toMsg(const urdf::Pose& in, geometry_msgs::Transform& out) {
    convert(in.position, out.translation);
    convert(in.rotation, out.rotation);
    return out;
}

}

// ============================================================================

namespace urdf {

inline void fromMsg(const geometry_msgs::Vector3& in, Vector3& out) {
    tf2::fromMsg(in, out);
}

inline void fromMsg(const geometry_msgs::Point32& in, Vector3& out) {
    tf2::fromMsg(in, out);
}

inline void fromMsg(const geometry_msgs::Quaternion& in, Rotation& out) {
    tf2::fromMsg(in, out);
}

inline void fromMsg(const geometry_msgs::Pose& in, Pose& out) {
    tf2::fromMsg(in, out);
}

inline void fromMsg(const geometry_msgs::Transform& in, Pose& out) {
    tf2::fromMsg(in, out);
}

inline geometry_msgs::Vector3 toMsg(const Vector3& in) {
    return tf2::toMsg(in);
}

inline geometry_msgs::Quaternion toMsg(const Rotation& in) {
    return tf2::toMsg(in);
}

inline geometry_msgs::Pose toMsg(const Pose& in) {
    return tf2::toMsg(in);
}

}

#endif
