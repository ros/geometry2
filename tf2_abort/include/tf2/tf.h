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

#ifndef TF2_TF_H
#define TF2_TF_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <sstream>
#include <map>

#include <tf2/exceptions.h>
#include "tf2/time_cache.h"
#include <boost/thread/mutex.hpp>
#include <boost/signals.hpp>

namespace tf2
{
/** \brief resolve tf2 names */
std::string resolve(const std::string& prefix, const std::string& frame_name);

enum ErrorValues { NO_ERROR = 0, LOOKUP_ERROR, CONNECTIVITY_ERROR, EXTRAPOLATION_ERROR};

/** \brief An internal representation of transform chains
 *
 * This struct is how the list of transforms are stored before being passed to computeTransformFromList. */
typedef struct
{
  std::vector<TransformStorage > inverseTransforms;
  std::vector<TransformStorage > forwardTransforms;
} TransformLists;

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
 * All function calls which pass frame ids can potentially throw the exception tf2::LookupException
 */
class Transformer
{
public:
  /************* Constants ***********************/
  static const unsigned int MAX_GRAPH_DEPTH = 100UL;   //!< The maximum number of time to recurse before assuming the tree has a loop.
  static const double DEFAULT_CACHE_TIME = 10.0;  //!< The default amount of time to cache data in seconds
  static const int64_t DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0ULL; //!< The default amount of time to extrapolate


  /** Constructor
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   *
   */
  Transformer(bool interpolating = true,
              ros::Duration cache_time_ = ros::Duration(DEFAULT_CACHE_TIME));
  virtual ~Transformer(void);

  /** \brief Clear all data */
  void clear();

  /** \brief Add transform information to the tf2 data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * returns true unless an error occured
   */
  bool setTransform(const StampedTransform& transform, const std::string & authority = "default_authority");

  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \param transform The transform reference to fill.  
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::MaxDepthException, tf2::ExtrapolationException
   */
  void lookupTransform(const std::string& target_frame, const std::string& source_frame,
                       const ros::Time& time, StampedTransform& transform) const;
  /** \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0 will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant in time. 
   * \param transform The transform reference to fill.  
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::MaxDepthException, tf2::ExtrapolationException
   */
  void lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                       const std::string& source_frame, const ros::Time& source_time,
                       const std::string& fixed_frame, StampedTransform& transform) const;

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
   * tf2::LookupException, tf2::ConnectivityException,
   * tf2::MaxDepthException, tf2::ExtrapolationException
   * 
   * New in geometry 1.1
   */

  void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
                   const tf::Point & reference_point, const std::string& reference_point_frame, 
                   const ros::Time& time, const ros::Duration& averaging_interval, 
                   geometry_msgs::Twist& twist) const;

  /** \brief lookup the twist of the tracking frame with respect to the observational frame 
   * 
   * This is a simplified version of
   * lookupTwist with it assumed that the reference point is the
   * origin of the tracking frame, and the reference frame is the
   * observation frame.  
   * 
   * New in geometry 1.1
   */

  void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, 
                   const ros::Time& time, const ros::Duration& averaging_interval,
                   geometry_msgs::Twist& twist) const;

  /** \brief Block until a transform is possible or it times out
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param timeout How long to block before failing
   * \param polling_sleep_duration How often to retest if failed
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool waitForTransform(const std::string& target_frame, const std::string& source_frame,
                        const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
                        std::string* error_msg = NULL) const;
  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool canTransform(const std::string& target_frame, const std::string& source_frame,
                    const ros::Time& time,
                    std::string* error_msg = NULL) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
                    const std::string& fixed_frame,
                    std::string* error_msg = NULL) const;

  /** \brief Block until a transform is possible or it times out
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param timeout How long to block before failing
   * \param polling_sleep_duration How often to retest if failed
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   */
  bool waitForTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
		    const std::string& fixed_frame,
                    const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
                    std::string* error_msg = NULL) const;


  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::MaxDepthException
   */
  std::string chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string & fixed_frame) const;

  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::MaxDepthException
   */
  void chainAsVector(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string & fixed_frame, std::vector<std::string>& output) const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  std::string allFramesAsString() const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  std::string allFramesAsDot() const;

  /** \brief A way to get a std::vector of available frame ids */
  void getFrameStrings(std::vector<std::string>& ids) const;

  /**@brief Fill the parent of a frame.
   * @param frame_id The frame id of the frame in question
   * @param parent The reference to the string to fill the parent
   * Returns true unless "NO_PARENT" */
  bool getParent(const std::string& frame_id, ros::Time time, std::string& parent) const;

  /**@brief Check if a frame exists in the tree
   * @param frame_id_str The frame id in question  */
  bool frameExists(const std::string& frame_id_str) const;

  /**@brief Set the distance which tf2 is allow to extrapolate
   * \param distance How far to extrapolate before throwing an exception
   * default is zero */
  void setExtrapolationLimit(const ros::Duration& distance);

  /**@brief Get the duration over which this transformer will cache */
  ros::Duration getCacheLength() { return cache_time;}

  /**
   * \brief Add a callback that happens when a new transform has arrived
   *
   * \param callback The callback, of the form void func();
   * \return A boost::signals::connection object that can be used to remove this
   * listener
   */
  boost::signals::connection addTransformsChangedListener(boost::function<void(void)> callback);
  void removeTransformsChangedListener(boost::signals::connection c);

  /** 
   * \brief Get the tf_prefix this is running with
   */
  std::string getTFPrefix() const { return tf_prefix_;};

  //Declare that it is safe to call waitForTransform
  void setUsingDedicatedThread(bool value) { using_dedicated_thread_ = value;};
  // Get the state of using_dedicated_thread_
  bool isUsingDedicatedThread() { return using_dedicated_thread_;};

protected:

  /** \brief The internal storage class for ReferenceTransform.
   *
   * An instance of this class is created for each frame in the system.
   * This class natively handles the relationship between frames.
   *
   * The derived class Pose3DCache provides a buffered history of positions
   * with interpolation.
   *
   */


  /******************** Internal Storage ****************/

  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  std::vector< TimeCache*> frames_;

  /** \brief A mutex to protect testing and allocating new frames */
  boost::mutex frame_mutex_;

  std::map<std::string, unsigned int> frameIDs_;
  std::map<unsigned int, std::string> frame_authority_;
  std::vector<std::string> frameIDs_reverse;

  /// How long to cache transform history
  ros::Duration cache_time;

  /// whether or not to interpolate or extrapolate
  bool interpolating;

  /// whether or not to allow extrapolation
  ros::Duration max_extrapolation_distance_;


  /// transform prefix to apply as necessary
  std::string tf_prefix_;

  typedef boost::signal<void(void)> TransformsChangedSignal;
  /// Signal which is fired whenever new transform data has arrived, from the thread the data arrived in
  TransformsChangedSignal transforms_changed_;
  boost::mutex transforms_changed_mutex_;

  //Whether it is safe to use waitForTransform.  This is basically stating that tf2 is multithreaded.  
  bool using_dedicated_thread_;
  
 public:
  // A flag to allow falling back to wall time
  bool fall_back_to_wall_time_;
  
 protected:
  /** Hack method to work around #4150 */
  ros::Time now() const { 
    if (!fall_back_to_wall_time_) 
      return ros::Time::now() ; 
    else {
      ros::WallTime wt = ros::WallTime::now();
      return ros::Time(wt.sec, wt.nsec);
    };
  }
   
  /************************* Internal Functions ****************************/

  /** \brief An accessor to get a frame, which will throw an exception if the frame is no there.
   * \param frame_number The frameID of the desired Reference Frame
   *
   * This is an internal function which will get the pointer to the frame associated with the frame id
   * Possible Exception: tf2::LookupException
   */
  TimeCache* getFrame(unsigned int frame_number) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  unsigned int lookupFrameNumber(const std::string& frameid_str) const
  {
    unsigned int retval = 0;
    boost::mutex::scoped_lock(frame_mutex_);
    std::map<std::string, unsigned int>::const_iterator map_it = frameIDs_.find(frameid_str);
    if (map_it == frameIDs_.end())
    {
      std::stringstream ss;
      ss << "Frame id " << frameid_str << " does not exist!";
      throw tf2::LookupException(ss.str());
    }
    else
      retval = map_it->second;
    return retval;
  };

  /// String to number for frame lookup with dynamic allocation of new frames
  unsigned int lookupOrInsertFrameNumber(const std::string& frameid_str)
  {
    unsigned int retval = 0;
    boost::mutex::scoped_lock(frame_mutex_);
    std::map<std::string, unsigned int>::iterator map_it = frameIDs_.find(frameid_str);
    if (map_it == frameIDs_.end())
    {
      retval = frames_.size();
      frameIDs_[frameid_str] = retval;
      frames_.push_back( new TimeCache(interpolating, cache_time, max_extrapolation_distance_));
      frameIDs_reverse.push_back(frameid_str);
    }
    else
      retval = frameIDs_[frameid_str];
    return retval;
  };
  ///Number to string frame lookup may throw LookupException if number invalid
  std::string lookupFrameString(unsigned int frame_id_num) const
  {
    if (frame_id_num >= frameIDs_reverse.size())
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
      throw LookupException(ss.str());
    }
    else
      return frameIDs_reverse[frame_id_num];

  };


  /** Find the list of connected frames necessary to connect two different frames */
  int lookupLists(unsigned int target_frame, ros::Time time, unsigned int source_frame, TransformLists & lists, std::string* error_string) const;

  bool test_extrapolation_one_value(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const;
  bool test_extrapolation_past(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const;
  bool test_extrapolation_future(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const;
  bool test_extrapolation(const ros::Time& target_time, const TransformLists& t_lists, std::string * error_string) const;

  /** Compute the transform based on the list of frames */
  btTransform computeTransformFromList(const TransformLists & list) const;

};



#endif //TF2_TF_H
