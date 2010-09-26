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

#include "LinearMath/btTransform.h"


using namespace tf2;

TimeCache::TimeCache( ros::Duration  max_storage_time,
                     ros::Duration max_extrapolation_time):
  max_storage_time_(max_storage_time),
  max_extrapolation_time_(max_extrapolation_time)
{};

bool TimeCache::getData(ros::Time time, TransformStorage & data_out) //returns false if data not available
{
  TransformStorage p_temp_1, p_temp_2;

  int num_nodes;
  ros::Duration time_diff;
  boost::mutex::scoped_lock lock(storage_lock_);

  ExtrapolationMode mode;
  num_nodes = findClosest(p_temp_1,p_temp_2, time, mode);
  if (num_nodes == 1)
  {
    data_out = p_temp_1;
    data_out.mode_ = mode;
  }
  else if (num_nodes == 2)
  {
    if( p_temp_1.frame_id_num_ == p_temp_2.frame_id_num_) 
    {
      interpolate(p_temp_1, p_temp_2, time, data_out);
      data_out.mode_ = mode;
    }
    else
    {
      data_out = p_temp_1;
      data_out.mode_ = mode;
    }       
  }
    
  return (num_nodes > 0);

};

bool TimeCache::insertData(const TransformStorage& new_data)
  {
    
    boost::mutex::scoped_lock lock(storage_lock_);
    
    std::list<TransformStorage >::iterator storage_it = storage_.begin();
    
    if(storage_it != storage_.end())
    {
      if (storage_it->header.stamp > new_data.header.stamp + max_storage_time_)
      {
        return false;
      }
    }
    
    
    while(storage_it != storage_.end())
    {
      if (storage_it->header.stamp <= new_data.header.stamp)
        break;
      storage_it++;
    }
    storage_.insert(storage_it, new_data);

    pruneList();
    return true;
  };


uint8_t TimeCache::findClosest(TransformStorage& one, TransformStorage& two, ros::Time target_time, ExtrapolationMode& mode)
{
  //No values stored
  if (storage_.empty())
  {
    return 0;
  }

  //If time == 0 return the latest
  if (target_time == ros::Time())
  {
    one = storage_.front();
    mode = ONE_VALUE;
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end())
  {
    one = *(storage_.begin());
    mode = ONE_VALUE;
    return 1;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  std::list<TransformStorage >::iterator storage_it = storage_.begin();
  while(storage_it != storage_.end())
  {
    if (storage_it->header.stamp <= target_time)
      break;
    storage_it++;
  }
  //Catch the case it is the first value in the list
  if (storage_it == storage_.begin())
  {
    one = *storage_it;
    two = *(++storage_it);
    mode = EXTRAPOLATE_FORWARD;

    /*    if (time_diff > max_extrapolation_time_) //Guarenteed in the future therefore positive
    {
      std::stringstream ss;
      ss << "Extrapolation Too Far in the future: target_time = "<< (target_time).toSec() <<", closest data at "
         << (one.header.stamp).toSec() << " and " << (two.header.stamp).toSec() <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_).toSec() <<" at "<< (target_time - one.header.stamp).toSec()<< " and " << (target_time - two.header.stamp).toSec() <<" respectively.";
      throw ExtrapolationException(ss.str());
    }
    */
    return 2;
  }

  //Catch the case where it's in the past
  if (storage_it == storage_.end())
  {
    one = *(--storage_it);
    two = *(--storage_it);
    mode = EXTRAPOLATE_BACK;
    /*
      time_diff = target_time - one.header.stamp;
    if (time_diff < ros::Duration()-max_extrapolation_time_) //Guarenteed in the past ///\todo check negative sign
    {
      std::stringstream ss;
      ss << "Extrapolation Too Far in the past: target_time = "<< (target_time).toSec() <<", closest data at "
         << (one.header.stamp).toSec() << " and " << (two.header.stamp).toSec() <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_).toSec() <<" at "<< (target_time - one.header.stamp).toSec()<< " and " << (target_time - two.header.stamp).toSec() <<" respectively."; //sign flip since in the past
      throw ExtrapolationException(ss.str());
    }
    */
    return 2;
  }

  //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  one = *(storage_it); //Older
  two = *(--storage_it); //Newer
  mode = INTERPOLATE;
  return 2;


};

void TimeCache::interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output)
{ 

  // Check for zero distance case 
  if( two.header.stamp == one.header.stamp )
  {
    output = two;
    return;    
  }
  //Calculate the ratio
  btScalar ratio = ((time - one.header.stamp).toSec()) / ((two.header.stamp - one.header.stamp).toSec());
  
  //Interpolate translation
  btVector3 v(0,0,0); //initialzed to fix uninitialized warning, not actually necessary
  v.setInterpolate3(btVector3(one.transform.translation.x, one.transform.translation.y, one.transform.translation.z), btVector3(two.transform.translation.x, two.transform.translation.y, two.transform.translation.z), ratio);
  output.transform.translation.x = v.getX();
  output.transform.translation.y = v.getY();
  output.transform.translation.z = v.getZ();
  
  //Interpolate rotation
  btQuaternion q1(one.transform.rotation.x, 
                  one.transform.rotation.y, 
                  one.transform.rotation.z, 
                  one.transform.rotation.w);
  
  btQuaternion q2(two.transform.rotation.x, 
                  two.transform.rotation.y, 
                  two.transform.rotation.z, 
                  two.transform.rotation.w);
  btQuaternion out;
  out = slerp( q1, q2 , ratio);
  output.transform.rotation.x = out.getX();
  output.transform.rotation.y = out.getY();
  output.transform.rotation.z = out.getZ();
  output.transform.rotation.w = out.getW();
  output.header.stamp = one.header.stamp;
  output.header.frame_id = one.header.frame_id;
  output.child_frame_id = one.child_frame_id;
  output.frame_id_num_ = one.frame_id_num_;
};

void TimeCache::clearList() {   boost::mutex::scoped_lock lock(storage_lock_); storage_.clear(); };

unsigned int TimeCache::getListLength() {   boost::mutex::scoped_lock lock(storage_lock_); return storage_.size(); };


ros::Time TimeCache::getLatestTimestamp() 
{   
  boost::mutex::scoped_lock lock(storage_lock_); 
  if (storage_.empty()) return ros::Time(); //empty list case
  return storage_.front().header.stamp; 
};

ros::Time TimeCache::getOldestTimestamp() 
{   
  boost::mutex::scoped_lock lock(storage_lock_); 
  if (storage_.empty()) return ros::Time(); //empty list case
  return storage_.back().header.stamp; 
};

void TimeCache::pruneList()
{
  ros::Time latest_time = storage_.begin()->header.stamp;
  
  while(!storage_.empty() && storage_.back().header.stamp + max_storage_time_ < latest_time)
  {
    storage_.pop_back();
  }
  
};
