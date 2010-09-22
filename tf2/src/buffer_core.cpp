/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "tf2/buffer_core.h"

//legacy
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

using namespace tf2;

BufferCore::BufferCore(ros::Duration cache_time): old_tf_(true, cache_time)
{

}

BufferCore::~BufferCore()
{

}

void BufferCore::clear()
{
  old_tf_.clear();
}

bool BufferCore::setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority)
{
  tf::StampedTransform tf_transform;
  tf::transformStampedMsgToTF(transform, tf_transform);
  return old_tf_.setTransform(tf_transform, authority);
};


geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame, 
                                                        const std::string& source_frame,
                                                        const ros::Time& time) const
{
  tf::StampedTransform t;
  old_tf_.lookupTransform(target_frame, source_frame, time, t);
  geometry_msgs::TransformStamped output;
  tf::transformStampedTFToMsg(t, output);
  return output;
};

                                                       
geometry_msgs::TransformStamped BufferCore::lookupTransform(const std::string& target_frame, 
                                                        const ros::Time& target_time,
                                                        const std::string& source_frame,
                                                        const ros::Time& source_time,
                                                        const std::string& fixed_frame) const
{
  tf::StampedTransform t;
  old_tf_.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, t);
  geometry_msgs::TransformStamped output;
  tf::transformStampedTFToMsg(t, output);
  return output;
};




geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const ros::Time& time, 
                                          const ros::Duration& averaging_interval) const
{
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, 
                      time, averaging_interval, t);
  return t;
};

geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const std::string& reference_frame,
                                          const tf::Point & reference_point, 
                                          const std::string& reference_point_frame, 
                                          const ros::Time& time, 
                                          const ros::Duration& averaging_interval) const
{
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, reference_frame, reference_point, reference_point_frame,
                      time, averaging_interval, t);
  return t;
};



bool BufferCore::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const ros::Time& time, std::string* error_msg) const
{
  return old_tf_.canTransform(target_frame, source_frame, time, error_msg);
}

bool BufferCore::canTransform(const std::string& target_frame, const ros::Time& target_time,
                          const std::string& source_frame, const ros::Time& source_time,
                          const std::string& fixed_frame, std::string* error_msg) const
{
  return old_tf_.canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, error_msg);
}
