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

#ifndef TF2_BUFFER_H
#define TF2_BUFFER_H

#include <tf2_cpp/buffer_interface.h>
#include <tf2/buffer_core.h>


namespace tf2
{

  // extend the BufferInterface class                                                                                                                      
  class Buffer: public BufferInterface
  {
  public:
    // lookup with timeout, simple api
    virtual geometry_msgs::TransformStamped 
      lookupTransform(const std::string& target_frame, const std::string& source_frame,
		      const ros::Time& time, const ros::Duration timeout = ros::Duration(0.0)) const;

    // lookup with timeout, advanced api
    virtual geometry_msgs::TransformStamped 
    lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		    const std::string& source_frame, const ros::Time& source_time,
		    const std::string& fixed_frame, const ros::Duration timeout = ros::Duration(0.0)) const;


    // can transform with timeout, simple api
    virtual bool
      canTransform(const std::string& target_frame, const std::string& source_frame, 
		   const ros::Time& target_time, const ros::Duration timeout = ros::Duration(0.0)) const;
    
    // can transform with timeout, advanced api
    virtual bool
      canTransform(const std::string& target_frame, const ros::Time& target_time,
		   const std::string& source_frame, const ros::Time& source_time,
		   const std::string& fixed_frame, const ros::Duration timeout = ros::Duration(0.0)) const;


  private:
    BufferCore buffer_core_;

  }; // class 
  
} // namespace

#endif // TF2_CPP_H
