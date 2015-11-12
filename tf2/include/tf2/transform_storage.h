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

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <iomanip>
#include <ctime>

namespace tf2
{

//TODO(tfoote) get a better solution for this

using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
using TempDuration = std::chrono::duration<double, std::nano>;
using IDuration = std::chrono::duration<int, std::nano>;
// This is the zero time in ROS
static const TimePoint TimePointZero = TimePoint(IDuration::zero());

inline TimePoint get_now()
{
  return std::chrono::system_clock::now();
}

inline double TempToSec(const TempDuration & input){
  return std::chrono::duration_cast<std::chrono::seconds>(input).count();
}

inline double TempToSec(const TimePoint& timepoint)
{
  return TempToSec(timepoint.time_since_epoch());
}

// Display functions as there is no default display
// TODO: find a proper way to handle display
inline std::string displayTimePoint(const TimePoint& stamp)
{
  // Below would only work with GCC 5.0 and above
  //return std::put_time(&stamp, "%c");
  std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::milliseconds>(stamp));
  return std::ctime(&time);
}
inline double displayDuration(const TempDuration& duration)
{
  return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
}

typedef uint32_t CompactFrameID;

/** \brief Storage for transforms and their parent */
class TransformStorage
{
public:
  TransformStorage();
  TransformStorage(const TimePoint& stamp, const Quaternion& q, const Vector3& t, CompactFrameID frame_id,
                   CompactFrameID child_frame_id);

  TransformStorage(const TransformStorage& rhs)
  {
    *this = rhs;
  }

  TransformStorage& operator=(const TransformStorage& rhs)
  {
#if 01
    rotation_ = rhs.rotation_;
    translation_ = rhs.translation_;
    stamp_ = rhs.stamp_;
    frame_id_ = rhs.frame_id_;
    child_frame_id_ = rhs.child_frame_id_;
#endif
    return *this;
  }

  tf2::Quaternion rotation_;
  tf2::Vector3 translation_;
  TimePoint stamp_;
  CompactFrameID frame_id_;
  CompactFrameID child_frame_id_;
};

}

#endif // TF2_TRANSFORM_STORAGE_H
