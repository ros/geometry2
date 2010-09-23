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

#ifndef TF2_TIME_CACHE_H
#define TF2_TIME_CACHE_H

#include <list>
#include <boost/thread/mutex.hpp>

#include "tf2/transform_datatypes.h"
#include "geometry_msgs/TransformStamped.h"

#include <sstream>

namespace tf2
{
enum ExtrapolationMode {  ONE_VALUE, INTERPOLATE, EXTRAPOLATE_BACK, EXTRAPOLATE_FORWARD };

/** \brief Storage for transforms and their parent */
class  TransformStorage : public geometry_msgs::TransformStamped
{
public:
  TransformStorage(){};
  TransformStorage(const geometry_msgs::TransformStamped& data, unsigned int frame_id_num): geometry_msgs::TransformStamped(data), frame_id_num_(frame_id_num){};
  unsigned int frame_id_num_;
  ExtrapolationMode mode_;
};


/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. */
class TimeCache
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage
  static const int64_t DEFAULT_MAX_EXTRAPOLATION_TIME = 0LL; //!< default max extrapolation of 0 nanoseconds \todo remove and make not optional??


  TimeCache(ros::Duration  max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME),
            ros::Duration  max_extrapolation_time = ros::Duration().fromNSec(DEFAULT_MAX_EXTRAPOLATION_TIME));

  bool getData(ros::Time time, TransformStorage & data_out); //returns false if data unavailable (should be thrown as lookup exception

  bool insertData(const TransformStorage& new_data);

  void interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output);  

  /** @brief Clear the list of stored values */
  void clearList();

  /** @brief Get the length of the stored list */
  unsigned int getListLength();

  /** @brief Get the latest timestamp cached */
  ros::Time getLatestTimestamp();

  /** @brief Get the oldest timestamp cached */
  ros::Time getOldestTimestamp();
private:
  std::list<TransformStorage > storage_;

  ros::Duration max_storage_time_;
  ros::Duration max_extrapolation_time_;

  boost::mutex storage_lock_;  ///!< The mutex to protect the linked list



  /// A helper function for getData
  //Assumes storage is already locked for it
  uint8_t findClosest(TransformStorage& one, TransformStorage& two, ros::Time target_time, ExtrapolationMode& mode);

  void pruneList();



};


}
#endif // TF2_TIME_CACHE_H
