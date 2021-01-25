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

#ifndef TF2_KDL_H
#define TF2_KDL_H

#include <tf2/convert.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


namespace tf2
{
/** \brief Convert a timestamped transform to the equivalent KDL data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an KDL Frame.
 */
inline
KDL::Frame transformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y,
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }

/** \brief Convert an KDL Frame to the equivalent geometry_msgs message type.
 * \param k The transform to convert, as an KDL Frame.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::TransformStamped kdlToTransform(const KDL::Frame& k)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = k.p.x();
  t.transform.translation.y = k.p.y();
  t.transform.translation.z = k.p.z();
  k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  return t;
}

// ---------------------
// Vector
// ---------------------
/** \brief Apply a geometry_msgs TransformStamped to an KDL-specific Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped KDL Vector data type.
 * \param t_out The transformed vector, as a timestamped KDL Vector data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Vector>& t_in, tf2::Stamped<KDL::Vector>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Vector>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

namespace impl
{
template <class Msg>
struct KDLVectorImplDetails
{
  /** \brief Convert a stamped KDL Vector type to a PointStamped message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped KDL Vector to convert.
   * \return The vector converted to a PointStamped message.
   */
  static void toMsg(const KDL::Vector& in, Msg& msg)
  {
    msg.x = in[0];
    msg.y = in[1];
    msg.z = in[2];
  }

  /** \brief Convert a PointStamped message type to a stamped KDL-specific Vector type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The PointStamped message to convert.
   * \param out The point converted to a timestamped KDL Vector.
   */
  static void fromMsg(const Msg& msg, KDL::Vector& out)
  {
    out[0] = msg.x;
    out[1] = msg.y;
    out[2] = msg.z;
  }
};

template <>
struct ImplDetails<KDL::Vector, geometry_msgs::Vector3> : KDLVectorImplDetails<geometry_msgs::Vector3>
{
};

template <>
struct ImplDetails<KDL::Vector, geometry_msgs::Point> : KDLVectorImplDetails<geometry_msgs::Point>
{
};

template <>
struct defaultMessage<KDL::Vector>
{
  using type = geometry_msgs::Point;
};

}  // namespace impl

// ---------------------
// Twist
// ---------------------
/** \brief Apply a geometry_msgs TransformStamped to an KDL-specific Twist type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The twist to transform, as a timestamped KDL Twist data type.
 * \param t_out The transformed Twist, as a timestamped KDL Frame data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Twist>& t_in, tf2::Stamped<KDL::Twist>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Twist>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }
namespace impl
{
template <>
struct ImplDetails<KDL::Twist, geometry_msgs::Twist>
{
  /** \brief Convert a stamped KDL Twist type to a TwistStamped message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped KDL Twist to convert.
   * \return The twist converted to a TwistStamped message.
   */
  static void toMsg(const KDL::Twist& in, geometry_msgs::Twist& msg)
  {
    msg.linear.x = in.vel[0];
    msg.linear.y = in.vel[1];
    msg.linear.z = in.vel[2];
    msg.angular.x = in.rot[0];
    msg.angular.y = in.rot[1];
    msg.angular.z = in.rot[2];
  }

  /** \brief Convert a TwistStamped message type to a stamped KDL-specific Twist type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The TwistStamped message to convert.
   * \param out The twist converted to a timestamped KDL Twist.
   */
  static void fromMsg(const geometry_msgs::Twist& msg, KDL::Twist& out)
  {
    out.vel[0] = msg.linear.x;
    out.vel[1] = msg.linear.y;
    out.vel[2] = msg.linear.z;
    out.rot[0] = msg.angular.x;
    out.rot[1] = msg.angular.y;
    out.rot[2] = msg.angular.z;
  }
};

template <>
struct defaultMessage<KDL::Twist>
{
  using type = geometry_msgs::Twist;
};

}  // namespace impl
// ---------------------
// Wrench
// ---------------------
/** \brief Apply a geometry_msgs TransformStamped to an KDL-specific Wrench type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The wrench to transform, as a timestamped KDL Wrench data type.
 * \param t_out The transformed Wrench, as a timestamped KDL Frame data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Wrench>& t_in, tf2::Stamped<KDL::Wrench>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Wrench>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

namespace impl
{
template <>
struct ImplDetails<KDL::Wrench, geometry_msgs::Wrench>
{
  /** \brief Convert a stamped KDL Wrench type to a WrenchStamped message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped KDL Wrench to convert.
   * \return The wrench converted to a WrenchStamped message.
   */
  static void toMsg(const KDL::Wrench& in, geometry_msgs::Wrench& msg)
  {
    msg.force.x = in.force[0];
    msg.force.y = in.force[1];
    msg.force.z = in.force[2];
    msg.torque.x = in.torque[0];
    msg.torque.y = in.torque[1];
    msg.torque.z = in.torque[2];
  }

  /** \brief Convert a WrenchStamped message type to a stamped KDL-specific Wrench type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The WrenchStamped message to convert.
   * \param out The wrench converted to a timestamped KDL Wrench.
   */
  static void fromMsg(const geometry_msgs::Wrench& msg, KDL::Wrench& out)
  {
    out.force[0] = msg.force.x;
    out.force[1] = msg.force.y;
    out.force[2] = msg.force.z;
    out.torque[0] = msg.torque.x;
    out.torque[1] = msg.torque.y;
    out.torque[2] = msg.torque.z;
  }
};

template <>
struct defaultMessage<KDL::Wrench>
{
  using type = geometry_msgs::Wrench;
};
}  // namespace impl
// ---------------------
// Frame
// ---------------------
/** \brief Apply a geometry_msgs TransformStamped to a KDL-specific Frame data type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The frame to transform, as a timestamped KDL Frame.
 * \param t_out The transformed frame, as a timestamped KDL Frame.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Frame>& t_in, tf2::Stamped<KDL::Frame>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Frame>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

namespace impl
{
template <>
struct ImplDetails<KDL::Frame, geometry_msgs::Pose>
{
  /** \brief Convert a stamped KDL Frame type to a Pose message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped KDL Frame to convert.
   * \return The frame converted to a Pose message.
   */
  static void toMsg(const KDL::Frame& in, geometry_msgs::Pose& msg)
  {
    msg.position.x = in.p[0];
    msg.position.y = in.p[1];
    msg.position.z = in.p[2];
    in.M.GetQuaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  }

  /** \brief Convert a Pose message type to a KDL Frame.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h.
   * \param msg The Pose message to convert.
   * \param out The pose converted to a KDL Frame.
   */
  static void fromMsg(const geometry_msgs::Pose& msg, KDL::Frame& out)
  {
    out.p[0] = msg.position.x;
    out.p[1] = msg.position.y;
    out.p[2] = msg.position.z;
    out.M = KDL::Rotation::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  }
};

template <>
struct defaultMessage<KDL::Frame>
{
  using type = geometry_msgs::Pose;
};
}  // namespace impl

} // namespace

#endif // TF2_KDL_H
