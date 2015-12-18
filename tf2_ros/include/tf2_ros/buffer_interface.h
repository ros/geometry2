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

#ifndef TF2_ROS_BUFFER_INTERFACE_H
#define TF2_ROS_BUFFER_INTERFACE_H

#include <tf2/buffer_core.h>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sstream>
//TODO(tfoote)  review removal #include <tf2/convert.h>

namespace tf2_ros
{
  // TODO(tfoote) replace this with something easier
  inline builtin_interfaces::msg::Time get_now_msg()
  {
    // TODO(tfoote) update to use an rclcpp now in future implementation
    // msg.header.stamp = builtin_interfaces::msg::Time.now();
    auto now = std::chrono::system_clock::now();
    std::chrono::nanoseconds ns = \
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    std::chrono::seconds s = \
      std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    builtin_interfaces::msg::Time t;
    t.sec = (int32_t)s.count();
    t.nanosec = (uint32_t)(ns.count() % 1000000000ull);
    return t;
  }

  inline double timeToSec(const builtin_interfaces::msg::Time & time_msg)
  {
    auto ns = std::chrono::duration<double, std::nano>(time_msg.nanosec);
    auto s = std::chrono::duration<double>(time_msg.sec);
    return (s + std::chrono::duration_cast<std::chrono::seconds>(ns)).count();
  }

// extend the TFCore class and the TFCpp class
class BufferInterface
{
public:

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  virtual geometry_msgs::msg::TransformStamped
    lookupTransform(const std::string& target_frame, const std::string& source_frame, 
		    const tf2::TimePoint& time, const tf2::Duration timeout) const = 0;

  /** \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0 will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant in time. 
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  virtual geometry_msgs::msg::TransformStamped 
    lookupTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
		    const std::string& source_frame, const tf2::TimePoint& source_time,
		    const std::string& fixed_frame, const tf2::Duration timeout) const = 0;


  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise 
   */
  virtual bool
    canTransform(const std::string& target_frame, const std::string& source_frame, 
		 const tf2::TimePoint& time, const tf2::Duration timeout, std::string* errstr = NULL) const = 0;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise 
   */
  virtual bool
    canTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
		 const std::string& source_frame, const tf2::TimePoint& source_time,
		 const std::string& fixed_frame, const tf2::Duration timeout, std::string* errstr = NULL) const = 0;


/* TODO(tfoote) restore transform methods
  // Transform, simple api, with pre-allocation
  template <class T>
    T& transform(const T& in, T& out, 
		 const std::string& target_frame, tf2::Duration timeout=tf2::Duration(0.0)) const
  {
    // do the transform
    tf2::doTransform(in, out, lookupTransform(target_frame, tf2::getFrameId(in), tf2::getTimestamp(in), timeout));
    return out;
  }
  
  
  // transform, simple api, no pre-allocation
  template <class T>
    T transform(const T& in, 
		const std::string& target_frame, tf2::Duration timeout=tf2::Duration(0.0)) const
  {
    T out;
    return transform(in, out, target_frame, timeout);
  }
  
  //transform, simple api, different types, pre-allocation
  template <class A, class B>
    B& transform(const A& in, B& out,
        const std::string& target_frame, tf2::Duration timeout=tf2::Duration(0.0)) const
  {
    A copy = transform(in, target_frame, timeout);
    tf2::convert(copy, out);
    return out;
  }
  
  // Transform, advanced api, with pre-allocation
  template <class T>
    T& transform(const T& in, T& out, 
		 const std::string& target_frame, const tf2::TimePoint& target_time,
		 const std::string& fixed_frame, tf2::Duration timeout=tf2::Duration(0.0)) const
  {
    // do the transform
    tf2::doTransform(in, out, lookupTransform(target_frame, target_time, 
                                              tf2::getFrameId(in), tf2::getTimestamp(in), 
                                              fixed_frame, timeout));
    return out;
  }


  // transform, advanced api, no pre-allocation
  template <class T>
    T transform(const T& in, 
		 const std::string& target_frame, const tf2::TimePoint& target_time,
		 const std::string& fixed_frame, tf2::Duration timeout=tf2::Duration(0.0)) const
  {
    T out;
    return transform(in, out, target_frame, target_time, fixed_frame, timeout);
  }
  
  // Transform, advanced api, different types, with pre-allocation
  template <class A, class B>
    B& transform(const A& in, B& out, 
		 const std::string& target_frame, const tf2::TimePoint& target_time,
		 const std::string& fixed_frame, tf2::Duration timeout=tf2::Duration(0.0)) const
  {
    // do the transform
    A copy = transform(in, target_frame, target_time, fixed_frame, timeout);
    tf2::convert(copy, out);
    return out;
  }
*/

 }; // class


} // namespace

#endif // TF2_ROS_BUFFER_INTERFACE_H
