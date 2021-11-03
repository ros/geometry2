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

#include "tf2/time_cache.h"
#include "tf2/exceptions.h"

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <assert.h>

namespace tf2 {

TransformStorage::TransformStorage()
{
}

TransformStorage::TransformStorage(const geometry_msgs::TransformStamped& data, CompactFrameID frame_id,
                                   CompactFrameID child_frame_id)
: stamp_(data.header.stamp)
, frame_id_(frame_id)
, child_frame_id_(child_frame_id)
{
  const geometry_msgs::Quaternion& o = data.transform.rotation;
  rotation_ = tf2::Quaternion(o.x, o.y, o.z, o.w);
  const geometry_msgs::Vector3& v = data.transform.translation;
  translation_ = tf2::Vector3(v.x, v.y, v.z);
}

TimeCache::TimeCache(ros::Duration max_storage_time)
: max_storage_time_(max_storage_time)
{}

namespace cache { // Avoid ODR collisions https://github.com/ros/geometry2/issues/175 
// hoisting these into separate functions causes an ~8% speedup.  Removing calling them altogether adds another ~10%
void createExtrapolationException1(ros::Time t0, ros::Time t1, std::string* error_str)
{
  if (error_str)
  {
    char str[116]; // Text without formatting strings has 76, each timestamp has up to 20
    snprintf(str, sizeof(str), "Lookup would require extrapolation at time %.09f, but only time %.09f is in the buffer", t0.toSec(), t1.toSec());
    *error_str = str;
  }
}

void createExtrapolationException2(ros::Time t0, ros::Time t1, std::string* error_str)
{
  if (error_str)
  {
    // Want this to come out positive, because this is a future extrapolation problem with t0
    // t0 needs to come first because it will be bigger than t1
    ros::Duration tdiff = t0 - t1;
    char str[163]; // Text without formatting strings has 102, each timestamp has up to 20
    snprintf(
        str, sizeof(str),
        "Lookup would require extrapolation %.09fs into the future.  Requested time %.09f but the latest data is at time %.09f",
        tdiff.toSec(), t0.toSec(), t1.toSec());
    *error_str = str;
  }
}

void createExtrapolationException3(ros::Time t0, ros::Time t1, std::string* error_str)
{
  if (error_str)
  {
    ros::Duration tdiff = t1 - t0;
    char str[163]; // Text without formatting strings has 102, each timestamp has up to 20
    snprintf(
        str, sizeof(str),
        "Lookup would require extrapolation %.09fs into the past.  Requested time %.09f but the earliest data is at time %.09f",
        tdiff.toSec(), t0.toSec(), t1.toSec());
    *error_str = str;
  }
}
} // namespace cache

bool operator>(const TransformStorage& lhs, const TransformStorage& rhs)
{
  return lhs.stamp_ > rhs.stamp_;
}

uint8_t TimeCache::findClosest(TransformStorage*& one, TransformStorage*& two, ros::Time target_time, std::string* error_str)
{
  //No values stored
  if (storage_.empty())
  {
    return 0;
  }

  //If time == 0 return the latest
  if (target_time.isZero())
  {
    one = &storage_.front();
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end())
  {
    TransformStorage& ts = *storage_.begin();
    if (ts.stamp_ == target_time)
    {
      one = &ts;
      return 1;
    }
    else
    {
      cache::createExtrapolationException1(target_time, ts.stamp_, error_str);
      return 0;
    }
  }

  ros::Time latest_time = (*storage_.begin()).stamp_;
  ros::Time earliest_time = (*(storage_.rbegin())).stamp_;

  if (target_time == latest_time)
  {
    one = &(*storage_.begin());
    return 1;
  }
  else if (target_time == earliest_time)
  {
    one = &(*storage_.rbegin());
    return 1;
  }
  // Catch cases that would require extrapolation
  else if (target_time > latest_time)
  {
    cache::createExtrapolationException2(target_time, latest_time, error_str);
    return 0;
  }
  else if (target_time < earliest_time)
  {
    cache::createExtrapolationException3(target_time, earliest_time, error_str);
    return 0;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  L_TransformStorage::iterator storage_it;
  TransformStorage storage_target_time;
  storage_target_time.stamp_ = target_time;

  storage_it = std::lower_bound(
      storage_.begin(),
      storage_.end(),
      storage_target_time, std::greater<TransformStorage>());

  //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  one = &*(storage_it); //Older
  two = &*(--storage_it); //Newer
  return 2;


}

void TimeCache::interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output)
{
  // Check for zero distance case
  if( two.stamp_ == one.stamp_ )
  {
    output = two;
    return;
  }
  //Calculate the ratio
  tf2Scalar ratio = (time - one.stamp_).toSec() / (two.stamp_ - one.stamp_).toSec();

  //Interpolate translation
  output.translation_.setInterpolate3(one.translation_, two.translation_, ratio);

  //Interpolate rotation
  output.rotation_ = slerp( one.rotation_, two.rotation_, ratio);

  output.stamp_ = time;
  output.frame_id_ = one.frame_id_;
  output.child_frame_id_ = one.child_frame_id_;
}

bool TimeCache::getData(ros::Time time, TransformStorage & data_out, std::string* error_str) //returns false if data not available
{
  TransformStorage* p_temp_1;
  TransformStorage* p_temp_2;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0)
  {
    return false;
  }
  else if (num_nodes == 1)
  {
    data_out = *p_temp_1;
  }
  else if (num_nodes == 2)
  {
    if( p_temp_1->frame_id_ == p_temp_2->frame_id_)
    {
      interpolate(*p_temp_1, *p_temp_2, time, data_out);
    }
    else
    {
      data_out = *p_temp_1;
    }
  }
  else
  {
    assert(0);
  }

  return true;
}

CompactFrameID TimeCache::getParent(ros::Time time, std::string* error_str)
{
  TransformStorage* p_temp_1;
  TransformStorage* p_temp_2;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0)
  {
    return 0;
  }

  return p_temp_1->frame_id_;
}

bool TimeCache::insertData(const TransformStorage& new_data, std::string* error_str)
{
  L_TransformStorage::iterator storage_it = storage_.begin();

  if(storage_it != storage_.end())
  {
    if (storage_it->stamp_ > new_data.stamp_ + max_storage_time_)
    {
      if (error_str)
      {
        *error_str = "TF_OLD_DATA ignoring data from the past (Possible reasons are listed at http://wiki.ros.org/tf/Errors%%20explained)";
      }
      return false;
    }
  }


  while(storage_it != storage_.end())
  {
    if (storage_it->stamp_ <= new_data.stamp_)
      break;
    storage_it++;
  }
  if (storage_it != storage_.end() && storage_it->stamp_ == new_data.stamp_)
  {
    if (error_str)
    {
      *error_str = "TF_REPEATED_DATA ignoring data with redundant timestamp";
    }
    return false;
  }
  else
  {
    storage_.insert(storage_it, new_data);
  }

  pruneList();
  return true;
}

void TimeCache::clearList()
{
  storage_.clear();
}

unsigned int TimeCache::getListLength()
{
  return storage_.size();
}

P_TimeAndFrameID TimeCache::getLatestTimeAndParent()
{
  if (storage_.empty())
  {
    return std::make_pair(ros::Time(), 0);
  }

  const TransformStorage& ts = storage_.front();
  return std::make_pair(ts.stamp_, ts.frame_id_);
}

ros::Time TimeCache::getLatestTimestamp() 
{   
  if (storage_.empty()) return ros::Time(); //empty list case
  return storage_.front().stamp_;
}

ros::Time TimeCache::getOldestTimestamp() 
{   
  if (storage_.empty()) return ros::Time(); //empty list case
  return storage_.back().stamp_;
}

void TimeCache::pruneList()
{
  ros::Time latest_time = storage_.begin()->stamp_;
  
  while(!storage_.empty() && storage_.back().stamp_ + max_storage_time_ < latest_time)
  {
    storage_.pop_back();
  }
  
} // namespace tf2
}
