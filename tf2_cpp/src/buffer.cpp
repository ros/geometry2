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


#include "tf2_cpp/buffer.h"

namespace tf2
{

geometry_msgs::TransformStamped 
Buffer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
			const ros::Time& time, const ros::Duration timeout) const
{
  // poll for transform if timeout is set
  if (timeout != ros::Duration(0.0)){
    ros::Time start_time = ros::Time::now();
    while (!buffer_core_.canTransform(target_frame, source_frame, time)){
      if (ros::Time::now() >= start_time + timeout){
	std::stringstream s;
	s << "Tried for " << timeout.toSec() << " seconds  to transform between " 
	  << source_frame << " and " << target_frame << " at time " << time.toSec();
	throw(TimeoutException(s.str().c_str()));
      }
      ros::Duration(0.01).sleep();
    }
  }
      // look up the transform
  return buffer_core_.lookupTransform(target_frame, source_frame, time);
}


geometry_msgs::TransformStamped 
Buffer::lookupTransform(const std::string& target_frame, const ros::Time& target_time,
			const std::string& source_frame, const ros::Time& source_time,
			const std::string& fixed_frame, const ros::Duration timeout) const
{
  // poll for transform if timeout is set
  if (timeout != ros::Duration(0.0)){
    ros::Time start_time = ros::Time::now();
    while (!buffer_core_.canTransform(target_frame, target_time, source_frame, source_time, fixed_frame)){
      if (ros::Time::now() >= start_time + timeout){
	std::stringstream s;
	s << "Tried for " << timeout.toSec() << " seconds  to transform between " 
	  << source_frame << " and " << fixed_frame << " at time " << source_time.toSec()
	  << " and between " << target_frame << " and " << fixed_frame << " at time " 
	  << target_time.toSec();
	throw(TimeoutException(s.str().c_str()));
      }
      ros::Duration(0.01).sleep();
    }
  }
  // look up the transform
  return buffer_core_.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}


}
