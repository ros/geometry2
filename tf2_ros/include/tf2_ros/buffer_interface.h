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
#include <geometry_msgs/TransformStamped.h>
#include <sstream>

namespace tf2
{
  
/**\brief The templated function expected to be able to do a transform
 *
 * This is the method which tf2 will use to try to apply a transform for any given datatype.   
 * \param data_in The data to be transformed.
 * \param data_out A reference to the output data.  Note this can point to data in and the method should be mutation safe.
 * \param transform The transform to apply to data_in to fill data_out.  
 * 
 * This method needs to be implemented by client library developers
 */
template <class T>
  void doTransform(const T& data_in, T& data_out, const geometry_msgs::TransformStamped& transform);

/**\brief Get the timestamp from data 
 * \param t The data input.
 * \return The timestamp associated with the data. 
 */
template <class T>
  const ros::Time& getTimestamp(const T& t);

/**\brief Get the frame_id from data 
 * \param t The data input.
 * \return The frame_id associated with the data. 
 */
template <class T>
  const std::string& getFrameId(const T& t);



/* An implementation for Stamped<P> datatypes */
template <class P>
  const ros::Time& getTimestamp(const tf2::Stamped<P>& t)
  {
    return t.stamp_;
  }

/* An implementation for Stamped<P> datatypes */
template <class P>
  const std::string& getFrameId(const tf2::Stamped<P>& t)
  {
    return t.frame_id_;
  }

template <class A, class B>
  void convert(const A& a, B& b)
  {
    //printf("In double type convert\n");
    fromMsg(toMsg(a), b);
  }

template <class A>
  void convert(const A& a1, A& a2)
  {
    //printf("In single type convert\n");
    if(&a1 != &a2)
      a2 = a1;
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
  virtual geometry_msgs::TransformStamped
    lookupTransform(const std::string& target_frame, const std::string& source_frame, 
		    const ros::Time& time, const ros::Duration timeout) const = 0;

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
  virtual geometry_msgs::TransformStamped 
    lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		    const std::string& source_frame, const ros::Time& source_time,
		    const std::string& fixed_frame, const ros::Duration timeout) const = 0;


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
		 const ros::Time& time, const ros::Duration timeout, std::string* errstr = NULL) const = 0;

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
    canTransform(const std::string& target_frame, const ros::Time& target_time,
		 const std::string& source_frame, const ros::Time& source_time,
		 const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr = NULL) const = 0;

  // Transform, simple api, with pre-allocation
  template <class T>
    T& transform(const T& in, T& out, 
		 const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    // do the transform
    doTransform(in, out, lookupTransform(target_frame, getFrameId(in), getTimestamp(in), timeout));
    return out;
  }


  // transform, simple api, no pre-allocation
  template <class T>
    T transform(const T& in, 
		const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    T out;
    return transform(in, out, target_frame, timeout);
  }

  //transform, simple api, different types, pre-allocation
  template <class A, class B>
    B& transform(const A& in, B& out,
        const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    A copy = transform(in, target_frame, timeout);
    convert(copy, out);
    return out;
  }

  // Transform, advanced api, with pre-allocation
  template <class T>
    T& transform(const T& in, T& out, 
		 const std::string& target_frame, const ros::Time& target_time,
		 const std::string& fixed_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    // do the transform
    doTransform(in, out, lookupTransform(target_frame, target_time, 
					     getFrameId(in), getTimestamp(in), 
					     fixed_frame, timeout));
    return out;
  }


  // transform, simple api, no pre-allocation
  template <class T>
    T& transform(const T& in, 
		 const std::string& target_frame, const ros::Time& target_time,
		 const std::string& fixed_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    T out;
    return transform(in, out, target_frame, target_time, fixed_frame, timeout);
  }

  // Transform, advanced api, different types, with pre-allocation
  template <class A, class B>
    B& transform(const A& in, B& out, 
		 const std::string& target_frame, const ros::Time& target_time,
		 const std::string& fixed_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    // do the transform
    A copy = transform(in, target_frame, target_time, fixed_frame, timeout);
    convert(copy, out);
    return out;
  }


 }; // class


} // namespace

#endif // TF2_ROS_BUFFER_INTERFACE_H
