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

#ifndef TF2_SENSOR_MSGS_H
#define TF2_SENSOR_MSGS_H

#include <tf2/convert.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace tf2
{

/********************/
/** PointCloud2    **/
/********************/

/** \brief Extract a timestamp from the header of a PointCloud2 message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PointCloud2 message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
const ros::Time& getTimestamp(const sensor_msgs::PointCloud2& p) {return p.header.stamp;}

/** \brief Extract a frame ID from the header of a PointCloud2 message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PointCloud2 message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
const std::string& getFrameId(const sensor_msgs::PointCloud2 &p) {return p.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const sensor_msgs::PointCloud2 &p_in, sensor_msgs::PointCloud2 &p_out, const geometry_msgs::TransformStamped& t_in)
{
  p_out = p_in;
  p_out.header = t_in.header;
  Eigen::Transform<float,3,Eigen::Isometry> t = Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
                                                                     t_in.transform.translation.z) * Eigen::Quaternion<float>(
                                                                     t_in.transform.rotation.w, t_in.transform.rotation.x,
                                                                     t_in.transform.rotation.y, t_in.transform.rotation.z);

  sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, "z");

  sensor_msgs::PointCloud2Iterator<float> x_out(p_out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(p_out, "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(p_out, "z");

  // Using individual matrix elements directly is apparently faster than relying on Eigen
  double r11 = t(0, 0);
  double r12 = t(0, 1);
  double r13 = t(0, 2);
  double r21 = t(1, 0);
  double r22 = t(1, 1);
  double r23 = t(1, 2);
  double r31 = t(2, 0);
  double r32 = t(2, 1);
  double r33 = t(2, 2);
  double t1 = t(0, 3);
  double t2 = t(1, 3);
  double t3 = t(2, 3);

  for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
    // Equivalent to "point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);"
    *x_out = *x_in * r11 + *y_in * r12 + *z_in * r13 + t1;
    *y_out = *x_in * r21 + *y_in * r22 + *z_in * r23 + t2;
    *z_out = *x_in * r31 + *y_in * r32 + *z_in * r33 + t3;
  }
}
inline
sensor_msgs::PointCloud2 toMsg(const sensor_msgs::PointCloud2 &in)
{
  return in;
}
inline
void fromMsg(const sensor_msgs::PointCloud2 &msg, sensor_msgs::PointCloud2 &out)
{
  out = msg;
}

} // namespace

#endif // TF2_SENSOR_MSGS_H
