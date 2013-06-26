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

#ifndef TF2_GEOMETRY_MSGS_H
#define TF2_GEOMETRY_MSGS_H

#include <tf2/convert.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>

namespace tf2
{
    
KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y, 
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }


/********************/
/** Vector3Stamped **/
/********************/

// method to extract timestamp from object
template <>
  const ros::Time& getTimestamp(const geometry_msgs::Vector3Stamped& t) {return t.header.stamp;}
  
// method to extract frame id from object
template <>
  const std::string& getFrameId(const geometry_msgs::Vector3Stamped& t) {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
  void doTransform(const geometry_msgs::Vector3Stamped& t_in, geometry_msgs::Vector3Stamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Stamped<KDL::Vector> v_out = tf2::Stamped<KDL::Vector>(gmTransformToKDL(transform).M * KDL::Vector(t_in.vector.x, t_in.vector.y, t_in.vector.z), 
								transform.header.stamp, transform.header.frame_id);
    t_out.vector.x = v_out[0];
    t_out.vector.y = v_out[1];
    t_out.vector.z = v_out[2];
    t_out.header.stamp = v_out.stamp_;
    t_out.header.frame_id = v_out.frame_id_;
  }
geometry_msgs::Vector3Stamped toMsg(const geometry_msgs::Vector3Stamped& in)
{
  return in;
}
void fromMsg(const geometry_msgs::Vector3Stamped& msg, geometry_msgs::Vector3Stamped& out)
{
  out = msg;
}



/******************/
/** PointStamped **/
/******************/

// method to extract timestamp from object
template <>
  const ros::Time& getTimestamp(const geometry_msgs::PointStamped& t)  {return t.header.stamp;}

// method to extract frame id from object
template <>
  const std::string& getFrameId(const geometry_msgs::PointStamped& t)  {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
  void doTransform(const geometry_msgs::PointStamped& t_in, geometry_msgs::PointStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Stamped<KDL::Vector> v_out = tf2::Stamped<KDL::Vector>(gmTransformToKDL(transform) * KDL::Vector(t_in.point.x, t_in.point.y, t_in.point.z), 
								transform.header.stamp, transform.header.frame_id);
    t_out.point.x = v_out[0];
    t_out.point.y = v_out[1];
    t_out.point.z = v_out[2];
    t_out.header.stamp = v_out.stamp_;
    t_out.header.frame_id = v_out.frame_id_;
  }
geometry_msgs::PointStamped toMsg(const geometry_msgs::PointStamped& in)
{
  return in;
}
void fromMsg(const geometry_msgs::PointStamped& msg, geometry_msgs::PointStamped& out)
{
  out = msg;
}


/*****************/
/** PoseStamped **/
/*****************/

// method to extract timestamp from object
template <>
  const ros::Time& getTimestamp(const geometry_msgs::PoseStamped& t)  {return t.header.stamp;}

// method to extract frame id from object
template <>
  const std::string& getFrameId(const geometry_msgs::PoseStamped& t)  {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
  void doTransform(const geometry_msgs::PoseStamped& t_in, geometry_msgs::PoseStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    KDL::Vector v(t_in.pose.position.x, t_in.pose.position.y, t_in.pose.position.z);
    KDL::Rotation r = KDL::Rotation::Quaternion(t_in.pose.orientation.x, t_in.pose.orientation.y, t_in.pose.orientation.z, t_in.pose.orientation.w);

    tf2::Stamped<KDL::Frame> v_out = tf2::Stamped<KDL::Frame>(gmTransformToKDL(transform) * KDL::Frame(r, v),
							      transform.header.stamp, transform.header.frame_id);
    t_out.pose.position.x = v_out.p[0];
    t_out.pose.position.y = v_out.p[1];
    t_out.pose.position.z = v_out.p[2];
    v_out.M.GetQuaternion(t_out.pose.orientation.x, t_out.pose.orientation.y, t_out.pose.orientation.z, t_out.pose.orientation.w);
    t_out.header.stamp = v_out.stamp_;
    t_out.header.frame_id = v_out.frame_id_;
  }
geometry_msgs::PoseStamped toMsg(const geometry_msgs::PoseStamped& in)
{
  return in;
}
void fromMsg(const geometry_msgs::PoseStamped& msg, geometry_msgs::PoseStamped& out)
{
  out = msg;
}




} // namespace

#endif // TF2_GEOMETRY_MSGS_H
