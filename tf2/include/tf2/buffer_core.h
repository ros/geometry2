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

/** \author Tully Foote */

#ifndef TF2_BUFFER_CORE_H
#define TF2_BUFFER_CORE_H

#include <string>

#include "ros/duration.h"
#include "ros/time.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/tf.h"

namespace tf2
{

/** \brief A Class which provides coordinate transforms between any two frames in a system.
 *
 * This class provides a simple interface to allow recording and lookup of
 * relationships between arbitrary frames of the system.
 *
 * libTF assumes that there is a tree of coordinate frame transforms which define the relationship between all coordinate frames.
 * For example your typical robot would have a transform from global to real world.  And then from base to hand, and from base to head.
 * But Base to Hand really is composed of base to shoulder to elbow to wrist to hand.
 * libTF is designed to take care of all the intermediate steps for you.
 *
 * Internal Representation
 * libTF will store frames with the parameters necessary for generating the transform into that frame from it's parent and a reference to the parent frame.
 * Frames are designated using an std::string
 * 0 is a frame without a parent (the top of a tree)
 * The positions of frames over time must be pushed in.
 *
 * All function calls which pass frame ids can potentially throw the exception tf::LookupException
 */
class BufferCore
{
public:
  /************* Constants ***********************/
  static const double DEFAULT_CACHE_TIME = 10.0;  //!< The default amount of time to cache data in seconds

  /** Constructor
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   *
   */
  BufferCore(ros::Duration cache_time_ = ros::Duration(DEFAULT_CACHE_TIME));
  virtual ~BufferCore(void);

  /** \brief Clear all data */
  void clear();

  /** \brief Add transform information to the tf data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * returns true unless an error occured
   */
  bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string & authority);

  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \param transform The transform reference to fill.  
   *
   * Possible exceptions tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException, tf::ExtrapolationException
   */
  geometry_msgs::TransformStamped 
    lookupTransform(const std::string& target_frame, const std::string& source_frame,
		    const ros::Time& time) const;
  /** \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0 will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant in time. 
   * \param transform The transform reference to fill.  
   *
   * Possible exceptions tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException, tf::ExtrapolationException
   */
  geometry_msgs::TransformStamped
    lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		    const std::string& source_frame, const ros::Time& source_time,
		    const std::string& fixed_frame) const;
  
  /** \brief Lookup the twist of the tracking_frame with respect to the observation frame in the reference_frame using the reference point
   * \param tracking_frame The frame to track
   * \param observation_frame The frame from which to measure the twist
   * \param reference_frame The reference frame in which to express the twist
   * \param reference_point The reference point with which to express the twist
   * \param reference_point_frame The frame_id in which the reference point is expressed
   * \param time The time at which to get the velocity
   * \param duration The period over which to average
   * \param twist The twist output
   * 
   * This will compute the average velocity on the interval 
   * (time - duration/2, time+duration/2). If that is too close to the most
   * recent reading, in which case it will shift the interval up to
   * duration/2 to prevent extrapolation.  Possible exceptions
   * tf::LookupException, tf::ConnectivityException,
   * tf::MaxDepthException, tf::ExtrapolationException
   * 
   * New in geometry 1.1
   */

  geometry_msgs::Twist
    lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
		const tf::Point & reference_point, const std::string& reference_point_frame, 
		const ros::Time& time, const ros::Duration& averaging_interval) const;

  /** \brief lookup the twist of the tracking frame with respect to the observational frame 
   * 
   * This is a simplified version of
   * lookupTwist with it assumed that the reference point is the
   * origin of the tracking frame, and the reference frame is the
   * observation frame.  
   * 
   * New in geometry 1.1
   */

  geometry_msgs::Twist
    lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, 
		const ros::Time& time, const ros::Duration& averaging_interval) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param timeout How long to block before failing
   * \param polling_sleep_duration How often to retest if failed
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool canTransform(const std::string& target_frame, const std::string& source_frame,
                    const ros::Time& time, std::string* error_msg = NULL) const;
  
  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param timeout How long to block before failing
   * \param polling_sleep_duration How often to retest if failed
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
                    const std::string& fixed_frame, std::string* error_msg = NULL) const;
  







private:

  //Using tf for now will be replaced fully
  tf::Transformer old_tf_;
};
  
}
#endif //TF2_CORE_H
