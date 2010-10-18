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
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "LinearMath/btTransform.h"

#include "tf2/time_cache.h"
//////////////////////////backwards startup for porting
//#include "tf/tf.h"

namespace tf2
{

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
 * All function calls which pass frame ids can potentially throw the exception tf::LookupException
 */
class BufferCore
{
public:
  /************* Constants ***********************/
  static const double DEFAULT_CACHE_TIME = 10.0;  //!< The default amount of time to cache data in seconds
  static const uint32_t MAX_GRAPH_DEPTH = 1000UL;  //!< The default amount of time to cache data in seconds
  static const int64_t DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0ULL; //!< The default amount of time to extrapolate

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
   * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
   * \return True unless an error occured
   */
  bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string & authority, bool is_static = false);

  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
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
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
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
   * \return twist The twist output
   * 
   * This will compute the average velocity on the interval 
   * (time - duration/2, time+duration/2). If that is too close to the most
   * recent reading, in which case it will shift the interval up to
   * duration/2 to prevent extrapolation.
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   * 
   * New in geometry 1.1
   */
  /*
  geometry_msgs::Twist
    lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
		const tf::Point & reference_point, const std::string& reference_point_frame, 
		const ros::Time& time, const ros::Duration& averaging_interval) const;
  */
  /** \brief lookup the twist of the tracking frame with respect to the observational frame 
   * 
   * This is a simplified version of
   * lookupTwist with it assumed that the reference point is the
   * origin of the tracking frame, and the reference frame is the
   * observation frame.  
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   * 
   * New in geometry 1.1
   */
  /*
  geometry_msgs::Twist
    lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, 
		const ros::Time& time, const ros::Duration& averaging_interval) const;
  */
  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise 
   */
  bool canTransform(const std::string& target_frame, const std::string& source_frame,
                    const ros::Time& time, std::string* error_msg = NULL) const;
  
  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise 
   */
  bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
                    const std::string& fixed_frame, std::string* error_msg = NULL) const;

  /** \brief A way to see what frames have been cached in yaml format
   * Useful for debugging tools
   */
  std::string allFramesAsYAML() const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  std::string allFramesAsString() const;
  
private:


  /******************** Internal Storage ****************/
  
  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  std::vector< TimeCacheInterface*> frames_;
  
  /** \brief A mutex to protect testing and allocating new frames on the above vector. */
  boost::mutex frame_mutex_;

  /** \brief A map from string frame ids to CompactFrameID */
  std::map<std::string, CompactFrameID> frameIDs_;
  /** \brief A map from CompactFrameID frame_id_numbers to string for debugging and output */
  std::vector<std::string> frameIDs_reverse;
  /** \brief A map to lookup the most recent authority for a given frame */
  std::map<CompactFrameID, std::string> frame_authority_;


  /// How long to cache transform history
  ros::Duration cache_time_;

  /// whether or not to allow extrapolation
  ros::Duration max_extrapolation_distance_;

  /************************* Internal Functions ****************************/

  /** \brief An accessor to get a frame, which will throw an exception if the frame is no there.
   * \param frame_number The frameID of the desired Reference Frame
   *
   * This is an internal function which will get the pointer to the frame associated with the frame id
   * Possible Exception: tf::LookupException
   */
  TimeCacheInterface* getFrame(CompactFrameID c_frame_id) const;

  TimeCacheInterface* allocateFrame(CompactFrameID cfid, bool is_static);


  bool warnFrameId(const std::string& function_name_arg, const std::string& frame_id) const;
  void validateFrameId(const std::string& function_name_arg, const std::string& frame_id) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookupFrameNumber(const std::string& frameid_str) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookupOrInsertFrameNumber(const std::string& frameid_str);

  ///Number to string frame lookup may throw LookupException if number invalid
  std::string lookupFrameString(CompactFrameID frame_id_num) const;

  /** Find the list of connected frames necessary to connect two different frames */
  int lookupLists(CompactFrameID target_frame, ros::Time time, CompactFrameID source_frame, TransformLists & lists, std::string* error_string) const;
  int findCommonParent(CompactFrameID frame1, CompactFrameID frame2, ros::Time time, std::string* error_string);

  bool test_extrapolation_one_value(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const;
  bool test_extrapolation_past(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const;
  bool test_extrapolation_future(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const;
  bool test_extrapolation(const ros::Time& target_time, const TransformLists& t_lists, std::string * error_string) const;

  /** Compute the transform based on the list of frames */
  btTransform computeTransformFromList(const TransformLists & list) const;

  void createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const;

  /**@brief Return the latest rostime which is common across the spanning set
   * zero if fails to cross */
  int getLatestCommonTime(const std::string& source, const std::string& dest, ros::Time& time, std::string * error_string) const;

  /** \brief convert Transform msg to Transform */
  static inline btTransform transformMsgToBT(const geometry_msgs::Transform& msg)
  {btTransform bt(btQuaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), btVector3(msg.translation.x, msg.translation.y, msg.translation.z)); return bt;};
  /** \brief convert Transform to Transform msg*/
  static inline geometry_msgs::Transform transformBTToMsg(const btTransform& bt)
  {
    geometry_msgs::Transform msg; 
    msg.translation.x = bt.getOrigin().getX();
    msg.translation.y = bt.getOrigin().getY();
    msg.translation.z = bt.getOrigin().getZ();
    msg.rotation.x = bt.getRotation().getX();
    msg.rotation.y = bt.getRotation().getY();
    msg.rotation.z = bt.getRotation().getZ();
    msg.rotation.w = bt.getRotation().getW();
    return msg;
  }


  /////////////////////////////////// Backwards hack for quick startup /////////////////////////
  //Using tf for now will be replaced fully
  //  tf::Transformer old_tf_;
};
  
}
#endif //TF2_CORE_H
