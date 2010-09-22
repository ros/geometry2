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

#ifndef TF2_CPP_H
#define TF2_CPP_H

#include <tf2/buffer_core.h>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>

namespace tf2
{
    
// this method needs to be implemented by client library developers
template <class T>
  void doTransform(const T& t_in, T& t_out, const geometry_msgs::TransformStamped& transform);

// method to extract timestamp from object
template <class T>
  const ros::Time& getTimestamp(const T& t);

// method to extract frame id from object
template <class T>
  const std::string& getFrameId(const T& t);

// method to extract timestamp from stamped object
template <class P>
  const ros::Time& getTimestamp(const tf2::Stamped<P>& t)
  {
    return t.stamp_;
  }

// method to extract frame id from object
template <class P>
  const std::string& getFrameId(const tf2::Stamped<P>& t)
  {
    return t.frame_id_;
  }



// extend the TFCore class and the TFCpp class
class BufferInterface
{
public:

  // lookup transform with timeout, simple api
  virtual geometry_msgs::TransformStamped
    lookupTransform(const std::string& target_frame, const std::string& source_frame, 
		    const ros::Time& target_time, const ros::Duration timeout = ros::Duration(0.0)) const = 0;


  // Transform, simple api, with pre-allocation
  template <class T>
    T& transform(const T& t_in, T& t_out, const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    // do the transform
    doTransform(t_in, t_out, lookupTransform(target_frame, getFrameId(t_in), getTimestamp(t_in), timeout));
    return t_out;
  }


  // transform, simple api, no pre-allocation
  template <class T>
    T transform(const T& t_in, const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    T t_out;
    return transform(t_in, t_out, target_frame, timeout);
  }


 }; // class


} // namespace

#endif // TF2_CPP_H
