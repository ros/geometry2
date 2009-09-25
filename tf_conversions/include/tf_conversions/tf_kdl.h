/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef CONVERSIONS_TF_KDL_H
#define CONVERSIONS_TF_KDL_H

#include "tf/transform_datatypes.h"
#include "kdl/frames.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

namespace tf
{
/// Converts a tf Vector3 into a KDL Vector
void VectorTFToKDL(const tf::Vector3& t, KDL::Vector& k);

/// Converts a tf Quaternion into a KDL Rotation
void RotationTFToKDL(const tf::Quaternion& t, KDL::Rotation& k);

/// Converts a tf Transform into a KDL Frame
void TransformTFToKDL(const tf::Transform &t, KDL::Frame &k);

/// Converts a tf Pose into a KDL Frame
void PoseTFToKDL(const tf::Pose& pose, KDL::Frame& frame);

/// Converts a KDL Frame into a tf Transform
void TransformKDLToTF(const KDL::Frame &k, tf::Transform &t);

/// Converts a KDL Frame into a tf Pose
void PoseKDLToTF(const KDL::Frame& frame, tf::Pose& pose);

/// Converts a KDL Twist into a Twist message
void TwistKDLToMsg(const KDL::Twist &t, geometry_msgs::Twist &m);

/// Converts a Twist message into a KDL Twist
void TwistMsgToKDL(const geometry_msgs::Twist &m, KDL::Twist &t);

/// Converts a Pose message into a KDL Frame
void PoseMsgToKDL(const geometry_msgs::Pose &p, KDL::Frame &t);

/// Converts a KDL Frame into a Pose message 
void PoseKDLToMsg(const KDL::Frame &t, geometry_msgs::Pose &p);



/* DEPRECATED FUNCTIONS */
/// Starting from a Pose from A to B, apply a Twist with reference frame A and reference point B, during a time t.
geometry_msgs::Pose addDelta(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist, const double &t)  __attribute__((deprecated));

}

#endif
