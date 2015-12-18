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


#include "tf2_ros/buffer.h"

#include <sstream>
#include <thread>

//TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_INFO printf


namespace tf2_ros
{

Buffer::Buffer(tf2::Duration cache_time, bool debug) :
  BufferCore(cache_time)
{
  // TODO(tfoote) reenable 
  // if(debug && !ros::service::exists("~tf2_frames", false))
  // {
  //   ros::NodeHandle n("~");
  //   frames_server_ = n.advertiseService("tf2_frames", &Buffer::getFrames, this);
  // }
}

geometry_msgs::msg::TransformStamped 
Buffer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                        const tf2::TimePoint& lookup_time, const tf2::Duration timeout) const
{
  canTransform(target_frame, source_frame, lookup_time, timeout);
  return lookupTransform(target_frame, source_frame, lookup_time);
}


geometry_msgs::msg::TransformStamped 
Buffer::lookupTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
                        const std::string& source_frame, const tf2::TimePoint& source_time,
                        const std::string& fixed_frame, const tf2::Duration timeout) const
{
  canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
  return lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

/** This is a workaround for the case that we're running inside of
    rospy and tf2::TimePoint is not initialized inside the c++ instance. 
    This makes the system fall back to Wall time if not initialized.  
*/
tf2::TimePoint now_fallback_to_wall()
{
  //TODO(tfoote) commented pending replacement for ros::time
  // try
  // {
    return tf2::get_now();
  //}
  // catch (ros::TimeNotInitializedException ex)
  // {
  //   //TODO(tfoote) should be walltime 
  //   ros::WallTime wt = get_now_msg(); // ros::WallTime::now(); 
  //   return ros::Time(wt.sec, wt.nsec); 
  // }
}

/** This is a workaround for the case that we're running inside of
    rospy and tf2::TimePoint is not initialized inside the c++ instance. 
    This makes the system fall back to Wall time if not initialized.  
    https://github.com/ros/geometry/issues/30
*/
void sleep_fallback_to_wall(const tf2::Duration& d)
{
  auto micro_d = std::chrono::duration_cast<std::chrono::microseconds>(d);
  std::this_thread::sleep_for(micro_d);
  //TODO(tfoote) reneable this when there's a new ros::Time equivalent
  // try
  // {
  //     d.sleep();
  // }
  // catch (ros::TimeNotInitializedException ex)
  // {
  //   ros::WallDuration wd = ros::WallDuration(d.sec, d.nsec); 
  //   wd.sleep();
  // }
}

void conditionally_append_timeout_info(std::string * errstr, const tf2::TimePoint& start_time,
                                       const tf2::Duration& timeout)
{
  if (errstr)
  {
    std::stringstream ss;
    ss << ". canTransform returned after "<< tf2::durationToSec(now_fallback_to_wall() - start_time) \
       <<" timeout was " << tf2::durationToSec(timeout) << ".";
    (*errstr) += ss.str();
  }
}

bool
Buffer::canTransform(const std::string& target_frame, const std::string& source_frame, 
                     const tf2::TimePoint& time, const tf2::Duration timeout, std::string* errstr) const
{
  if (!checkAndErrorDedicatedThreadPresent(errstr))
    return false;

  // poll for transform if timeout is set
  tf2::TimePoint start_time = now_fallback_to_wall();
  while (now_fallback_to_wall() < start_time + timeout && 
         !canTransform(target_frame, source_frame, time) &&
         (now_fallback_to_wall()+tf2::Duration(3.0) >= start_time) &&  //don't wait when we detect a bag loop
         (rclcpp::ok()// || !ros::isInitialized() //TODO(tfoote) restore
       )) // Make sure we haven't been stopped (won't work for pytf)
    {
      sleep_fallback_to_wall(tf2::Duration(0.01));
    }
  bool retval = canTransform(target_frame, source_frame, time, errstr);
  conditionally_append_timeout_info(errstr, start_time, timeout);
  return retval;
}

    
bool
Buffer::canTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
                     const std::string& source_frame, const tf2::TimePoint& source_time,
                     const std::string& fixed_frame, const tf2::Duration timeout, std::string* errstr) const
{
  if (!checkAndErrorDedicatedThreadPresent(errstr))
    return false;

  // poll for transform if timeout is set
  tf2::TimePoint start_time = now_fallback_to_wall();
  while (now_fallback_to_wall() < start_time + timeout && 
         !canTransform(target_frame, target_time, source_frame, source_time, fixed_frame) &&
         (now_fallback_to_wall()+tf2::Duration(3.0) >= start_time) &&  //don't wait when we detect a bag loop
         (rclcpp::ok() //|| !ros::isInitialized() //TODO(tfoote) restore
          )
        ) // Make sure we haven't been stopped (won't work for pytf)
         {  
           sleep_fallback_to_wall(tf2::Duration(0.01));
         }
  bool retval = canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, errstr);
  conditionally_append_timeout_info(errstr, start_time, timeout);
  return retval; 
}


bool Buffer::getFrames(tf2_msgs::srv::FrameGraph::Request& req, tf2_msgs::srv::FrameGraph::Response& res) 
{
  res.frame_yaml = allFramesAsYAML();
  return true;
}



bool Buffer::checkAndErrorDedicatedThreadPresent(std::string* error_str) const
{
  if (isUsingDedicatedThread())
    return true;
  


  if (error_str)
    *error_str = tf2_ros::threading_error;

  ROS_ERROR("%s", tf2_ros::threading_error.c_str());
  return false;
}



}
