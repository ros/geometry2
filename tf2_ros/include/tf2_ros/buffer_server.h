/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef TF2_ROS_BUFFER_SERVER_H_
#define TF2_ROS_BUFFER_SERVER_H_

#include <actionlib/server/action_server.h>
#include <tf2_msgs/LookupTransformAction.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/buffer.h>

namespace tf2_ros
{
  /** \brief Action server for the actionlib-based implementation of tf2_ros::BufferInterface.
   * 
   * Use this class with a tf2_ros::TransformListener in the same process.
   * You can use this class with a tf2_ros::BufferClient in a different process.
   */
  class BufferServer
  {
    private:
      typedef actionlib::ActionServer<tf2_msgs::LookupTransformAction> LookupTransformServer;
      typedef LookupTransformServer::GoalHandle GoalHandle;

      struct GoalInfo
      {
        GoalHandle handle;
        tf2::TimePoint end_time;
      };

    public:
      /** \brief Constructor
       * \param buffer The Buffer that this BufferServer will wrap.
       * \param ns The namespace in which to look for action clients.
       * \param auto_start Pass argument to the constructor of the ActionServer.
       * \param check_period How often to check for changes to known transforms (via a timer event).
       */
      BufferServer(const Buffer& buffer, const std::string& ns,
          bool auto_start = true, tf2::Duration check_period = tf2::Duration(0.01));

      /** \brief Start the action server.
       */
      void start();

    private:
      void goalCB(GoalHandle gh);
      void cancelCB(GoalHandle gh);
      void checkTransforms(const tf2::TimePointrEvent& e);
      bool canTransform(GoalHandle gh);
      geometry_msgs::TransformStamped lookupTransform(GoalHandle gh);

      const Buffer& buffer_;
      LookupTransformServer server_;
      std::list<GoalInfo> active_goals_;
      std::mutex mutex_;
      tf2::TimePointr check_timer_;
  };
}
#endif
