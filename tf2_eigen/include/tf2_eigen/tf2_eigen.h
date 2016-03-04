/*
 * Copyright (c) Koji Terada
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

/** \author Koji Terada */

#ifndef TF2_EIGEN_H
#define TF2_EIGEN_H

#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>


namespace tf2
{

inline    
Eigen::Affine3d transformToEigen(const geometry_msgs::TransformStamped& t) {
  return Eigen::Affine3d(Eigen::Translation3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
			 * Eigen::Quaterniond(t.transform.rotation.w, 
					      t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z));
}

inline
geometry_msgs::TransformStamped eigenToTransform(const Eigen::Affine3d& T)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = T.translation().x();
  t.transform.translation.y = T.translation().y();
  t.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.rotation());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  
  return t;
}


// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Vector3d>& t_in, 
		 tf2::Stamped<Eigen::Vector3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Vector3d>(transformToEigen(transform) * t_in,
					transform.header.stamp, 
					transform.header.frame_id);
}

//convert to vector message
inline
geometry_msgs::PointStamped toMsg(const tf2::Stamped<Eigen::Vector3d>& in)
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.point.x = in.x();
  msg.point.y = in.y();
  msg.point.z = in.z();
  return msg;
}

inline
void fromMsg(const geometry_msgs::PointStamped& msg, tf2::Stamped<Eigen::Vector3d>& out) {
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.x() = msg.point.x;
  out.y() = msg.point.y;
  out.z() = msg.point.z;
}


// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Affine3d>& t_in,
		 tf2::Stamped<Eigen::Affine3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Affine3d>(transformToEigen(transform) * t_in, transform.header.stamp, transform.header.frame_id);
}

//convert to pose message
inline
geometry_msgs::PoseStamped toMsg(const tf2::Stamped<Eigen::Affine3d>& in)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.pose.position.x = in.translation().x();
  msg.pose.position.y = in.translation().y();
  msg.pose.position.z = in.translation().z();
  msg.pose.orientation.x = Eigen::Quaterniond(in.rotation()).x();
  msg.pose.orientation.y = Eigen::Quaterniond(in.rotation()).y();
  msg.pose.orientation.z = Eigen::Quaterniond(in.rotation()).z();
  msg.pose.orientation.w = Eigen::Quaterniond(in.rotation()).w();
  return msg;
}

inline
void fromMsg(const geometry_msgs::PoseStamped& msg, tf2::Stamped<Eigen::Affine3d>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.setData(Eigen::Affine3d(Eigen::Translation3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
			       * Eigen::Quaterniond(msg.pose.orientation.w, 
						    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)));
}

} // namespace

#endif // TF2_EIGEN_H
