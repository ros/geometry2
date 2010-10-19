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

/** \author Tully Foote */

#ifndef TF2_TRANSFORM_STORAGE_H
#define TF2_TRANSFORM_STORAGE_H

#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include <ros/message_forward.h>
#include <ros/time.h>
#include <ros/types.h>

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(TransformStamped);
}

namespace tf2
{
enum ExtrapolationMode {  ONE_VALUE, INTERPOLATE, EXTRAPOLATE_BACK, EXTRAPOLATE_FORWARD };

typedef uint32_t CompactFrameID;

/** \brief Storage for transforms and their parent */
class  TransformStorage
{
public:
  TransformStorage();
  TransformStorage(const geometry_msgs::TransformStamped& data, CompactFrameID frame_id, CompactFrameID child_frame_id);
  btQuaternion rotation_;
  btVector3 translation_;
  ros::Time stamp_;
  CompactFrameID frame_id_;
  CompactFrameID child_frame_id_;
  ExtrapolationMode mode_;
};

}

#endif // TF2_TRANSFORM_STORAGE_H

