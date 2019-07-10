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

/** \brief Apply the given isometry transform to an xyz-channel in the pointcloud.
 * \param p_in Input pointcloud.
 * \param p_out Output pointcloud (can be the same as input).
 * \param t The transform to apply.
 * \param channelPrefix Channel name prefix. If prefix is e.g. "vp_", then channels "vp_x", "vp_y" and "vp_z" will be considered.
 *                      Empty prefix denotes the "x", "y" and "z" channels of point positions.
 * \param onlyRotation If true, only rotation will be applied (i.e. the channel is just a directional vector).
 */
inline void transformChannel(const sensor_msgs::PointCloud2 &p_in,
    sensor_msgs::PointCloud2 &p_out, const Eigen::Isometry3f& t,
    const std::string& channelPrefix, bool onlyRotation = false)
{
  sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, channelPrefix + "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, channelPrefix + "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, channelPrefix + "z");

  sensor_msgs::PointCloud2Iterator<float> x_out(p_out, channelPrefix + "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(p_out, channelPrefix + "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(p_out, channelPrefix + "z");

  Eigen::Vector3f point;
  for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
    if (!onlyRotation)
      point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);
    else
      point = t.linear() * Eigen::Vector3f(*x_in, *y_in, *z_in);

    *x_out = point.x();
    *y_out = point.y();
    *z_out = point.z();
  }
}

/** \brief Transform given 3D-data channels in the pointcloud using the given transformation.
 * \param p_in Input point cloud.
 * \param p_outOutput pointcloud (can be the same as input).
 * \param t_in The transform to apply.
 * \param n_channels Number of channels to transform.
 * \param ... Channels are given as pairs (char* channelPrefix, int onlyRotation),
 *            where the channelPrefix is the channel prefix passed further to transformChannel()
 *            and onlyRotation specifies whether the whole transform should be applied or only
 *            its rotation part (useful for transformation of directions, i.e. normals).
 */
inline void doTransformChannels(const sensor_msgs::PointCloud2 &p_in, sensor_msgs::PointCloud2 &p_out, const geometry_msgs::TransformStamped& t_in, int n_channels, ...)
{
  p_out = p_in;
  p_out.header = t_in.header;
  Eigen::Transform<float,3,Eigen::Isometry> t = Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
                                                                     t_in.transform.translation.z) * Eigen::Quaternion<float>(
                                                                     t_in.transform.rotation.w, t_in.transform.rotation.x,
                                                                     t_in.transform.rotation.y, t_in.transform.rotation.z);

  // transform the positional channels like points, viewpoints and normals
  va_list vl;
  va_start(vl, n_channels);
  for (int i = 0; i < n_channels; ++i) {
    const std::string prefix(va_arg(vl, char *));
    const bool onlyRotation = static_cast<bool>(va_arg(vl, int));
    const auto xField = prefix + "x";

    for (const auto &field : p_in.fields) {
      if (field.name == xField)
        transformChannel(p_in, p_out, t, prefix, onlyRotation);
    }
  }
  va_end(vl);
}

// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const sensor_msgs::PointCloud2 &p_in, sensor_msgs::PointCloud2 &p_out, const geometry_msgs::TransformStamped& t_in)
{
  doTransformChannels(p_in, p_out, t_in,
      3,
      "", false,  // points
      "vp_", false, // viewpoints
      "normal_", true // normals
  );
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
