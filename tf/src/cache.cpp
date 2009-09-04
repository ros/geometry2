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

#include "tf/time_cache.h"

using namespace tf;

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
    if(interpolating_ && ( p_temp_1.parent_frame_id == p_temp_2.parent_frame_id) ) // if we're interpolating and haven't reparented
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
    if (storage_it->stamp_ <= target_time)
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
         << (one.stamp_).toSec() << " and " << (two.stamp_).toSec() <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_).toSec() <<" at "<< (target_time - one.stamp_).toSec()<< " and " << (target_time - two.stamp_).toSec() <<" respectively.";
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
      time_diff = target_time - one.stamp_;
    if (time_diff < ros::Duration()-max_extrapolation_time_) //Guarenteed in the past ///\todo check negative sign
    {
      std::stringstream ss;
      ss << "Extrapolation Too Far in the past: target_time = "<< (target_time).toSec() <<", closest data at "
         << (one.stamp_).toSec() << " and " << (two.stamp_).toSec() <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_).toSec() <<" at "<< (target_time - one.stamp_).toSec()<< " and " << (target_time - two.stamp_).toSec() <<" respectively."; //sign flip since in the past
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
  if( two.stamp_ == one.stamp_ )
  {
    output = two;
    return;    
  }
  //Calculate the ratio
  btScalar ratio = ((time - one.stamp_).toSec()) / ((two.stamp_ - one.stamp_).toSec());
  
  //Interpolate translation
  btVector3 v(0,0,0); //initialzed to fix uninitialized warning, not actually necessary
  v.setInterpolate3(one.getOrigin(), two.getOrigin(), ratio);
  output.setOrigin(v);
  
  //Interpolate rotation
  btQuaternion q1,q2;
  one.getBasis().getRotation(q1);
  two.getBasis().getRotation(q2);
  output.setRotation(slerp( q1, q2 , ratio));
  output.stamp_ = one.stamp_;
  output.frame_id_ = one.frame_id_;
  output.parent_id_ = one.parent_id_;
  output.parent_frame_id = one.parent_frame_id;
};

