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
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>


namespace tf2
{

/** \brief Convert a timestamped transform to the equivalent Eigen data type.
 * \param t The transform to convert, as a geometry_msgs Transform message.
 * \return The transform message converted to an Eigen Isometry3d transform.
 */
 inline
 Eigen::Isometry3d transformToEigen(const geometry_msgs::Transform& t) {
 return Eigen::Isometry3d(Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z)
			 * Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z));
}

/** \brief Convert a timestamped transform to the equivalent Eigen data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an Eigen Isometry3d transform.
 */
inline
Eigen::Isometry3d transformToEigen(const geometry_msgs::TransformStamped& t) {
  return transformToEigen(t.transform);
}

/** \brief Convert an Eigen Affine3d transform to the equivalent geometry_msgs message type.
 * \param T The transform to convert, as an Eigen Affine3d transform.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::TransformStamped eigenToTransform(const Eigen::Affine3d& T)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = T.translation().x();
  t.transform.translation.y = T.translation().y();
  t.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.linear());  // assuming that upper 3x3 matrix is orthonormal
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  return t;
}

/** \brief Convert an Eigen Isometry3d transform to the equivalent geometry_msgs message type.
 * \param T The transform to convert, as an Eigen Isometry3d transform.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::TransformStamped eigenToTransform(const Eigen::Isometry3d& T)
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

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Vector3d type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * functions rely on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The vector to transform, as a Eigen Vector3d data type.
 * \param t_out The transformed vector, as a Eigen Vector3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const Eigen::Vector3d& t_in, Eigen::Vector3d& t_out, const geometry_msgs::TransformStamped& transform)
{
  t_out = Eigen::Vector3d(transformToEigen(transform) * t_in);
}

namespace impl
{
template <class Message>
struct Vector3ImplDetails
{
  /** \brief Convert a Eigen Vector3d type to a Point message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The timestamped Eigen Vector3d to convert.
   * \return The vector converted to a Point message.
   */
  static void toMsg(const Eigen::Vector3d& in, Message& msg)
  {
    msg.x = in.x();
    msg.y = in.y();
    msg.z = in.z();
  }

  /** \brief Convert a Point message type to a Eigen-specific Vector3d type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The Point message to convert.
   * \param out The point converted to a Eigen Vector3d.
   */
  static void fromMsg(const Message& msg, Eigen::Vector3d& out)
  {
    out.x() = msg.x;
    out.y() = msg.y;
    out.z() = msg.z;
  }
};

template <>
struct ImplDetails<Eigen::Vector3d, geometry_msgs::Point> : public Vector3ImplDetails<geometry_msgs::Point>
{
};

template <>
struct ImplDetails<Eigen::Vector3d, geometry_msgs::Vector3> : public Vector3ImplDetails<geometry_msgs::Vector3>
{
};

template <>
struct defaultMessage<Eigen::Vector3d>
{
  using type = geometry_msgs::Point;
};
}  // namespace impl

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Vector3d type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Eigen Vector3d data type.
 * \param t_out The transformed vector, as a timestamped Eigen Vector3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Vector3d>& t_in,
		 tf2::Stamped<Eigen::Vector3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Vector3d>(transformToEigen(transform) * t_in,
					transform.header.stamp,
					transform.header.frame_id);
}

/** \brief Apply a geometry_msgs Transform to an Eigen Affine3d transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * function relies on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The frame to transform, as a Eigen Affine3d transform.
 * \param t_out The transformed frame, as a Eigen Affine3d transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const Eigen::Affine3d& t_in,
                 Eigen::Affine3d& t_out,
                 const geometry_msgs::TransformStamped& transform) {
  t_out = Eigen::Affine3d(transformToEigen(transform) * t_in);
}

template <>
inline
void doTransform(const Eigen::Isometry3d& t_in,
                 Eigen::Isometry3d& t_out,
                 const geometry_msgs::TransformStamped& transform) {
  t_out = Eigen::Isometry3d(transformToEigen(transform) * t_in);
}

namespace impl
{
/** \brief Convert a Eigen Quaterniond type to a Quaternion message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The Eigen Quaterniond to convert.
 * \return The quaternion converted to a Quaterion message.
 */
template <>
struct ImplDetails<Eigen::Quaterniond, geometry_msgs::Quaternion>
{
  static void toMsg(const Eigen::Quaterniond& in, geometry_msgs::Quaternion& msg)
  {
    msg.w = in.w();
    msg.x = in.x();
    msg.y = in.y();
    msg.z = in.z();
  }

  /** \brief Convert a Quaternion message type to a Eigen-specific Quaterniond type.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h
   * \param msg The Quaternion message to convert.
   * \param out The quaternion converted to a Eigen Quaterniond.
   */
  static void fromMsg(const geometry_msgs::Quaternion& msg, Eigen::Quaterniond& out)
  {
    out = Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
  }
};

template <>
struct defaultMessage<Eigen::Quaterniond>
{
  using type = geometry_msgs::Quaternion;
};
}  // namespace impl

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Quaterniond type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * functions rely on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The vector to transform, as a Eigen Quaterniond data type.
 * \param t_out The transformed vector, as a Eigen Quaterniond data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(const Eigen::Quaterniond& t_in,
                 Eigen::Quaterniond& t_out,
                 const geometry_msgs::TransformStamped& transform) {
  Eigen::Quaterniond t;
  fromMsg(transform.transform.rotation, t);
  t_out = t.inverse() * t_in * t;
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Quaterniond type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Eigen Quaterniond data type.
 * \param t_out The transformed vector, as a timestamped Eigen Quaterniond data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Quaterniond>& t_in,
     tf2::Stamped<Eigen::Quaterniond>& t_out,
     const geometry_msgs::TransformStamped& transform) {
  t_out.frame_id_ = transform.header.frame_id;
  t_out.stamp_ = transform.header.stamp;
  doTransform(static_cast<const Eigen::Quaterniond&>(t_in), static_cast<Eigen::Quaterniond&>(t_out), transform);
}

namespace impl
{
template <typename T>
struct PoseImplDetails
{
  /** \brief Convert a Eigen Affine3d transform type to a Pose message.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in The Eigen Affine3d to convert.
   * \return The Eigen transform converted to a Pose message.
   */
  static void toMsg(const T& in, geometry_msgs::Pose& msg)
  {
    msg.position.x = in.translation().x();
    msg.position.y = in.translation().y();
    msg.position.z = in.translation().z();
    const Eigen::Quaterniond q(in.linear());
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    if (msg.orientation.w < 0)
    {
      msg.orientation.x *= -1;
      msg.orientation.y *= -1;
      msg.orientation.z *= -1;
      msg.orientation.w *= -1;
    }
  }

  /** \brief Convert a Pose message transform type to a Eigen Affine3d.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param msg The Pose message to convert.
   * \param out The pose converted to a Eigen Affine3d.
   */
  static void fromMsg(const geometry_msgs::Pose& msg, T& out)
  {
    out = T(Eigen::Translation3d(msg.position.x, msg.position.y, msg.position.z) *
            Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z));
  }
};

template <>
struct ImplDetails<Eigen::Affine3d, geometry_msgs::Pose> : public PoseImplDetails<Eigen::Affine3d>
{
};

template <>
struct ImplDetails<Eigen::Isometry3d, geometry_msgs::Pose> : public PoseImplDetails<Eigen::Isometry3d>
{
};

template <>
struct defaultMessage<Eigen::Affine3d>
{
  using type = geometry_msgs::Pose;
};

template <>
struct defaultMessage<Eigen::Isometry3d>
{
  using type = geometry_msgs::Pose;
};
/** \brief Convert a Eigen 6x1 Matrix type to a Twist message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The 6x1 Eigen Matrix to convert.
 * \return The Eigen Matrix converted to a Twist message.
 */
template <>
struct ImplDetails<Eigen::Matrix<double, 6, 1>, geometry_msgs::Twist>
{
  static void toMsg(const Eigen::Matrix<double, 6, 1>& in, geometry_msgs::Twist& msg)
  {
    msg.linear.x = in[0];
    msg.linear.y = in[1];
    msg.linear.z = in[2];
    msg.angular.x = in[3];
    msg.angular.y = in[4];
    msg.angular.z = in[5];
  }

  /** \brief Convert a Twist message transform type to a Eigen 6x1 Matrix.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param msg The Twist message to convert.
   * \param out The twist converted to a Eigen 6x1 Matrix.
   */
  static void fromMsg(const geometry_msgs::Twist& msg, Eigen::Matrix<double, 6, 1>& out)
  {
    out[0] = msg.linear.x;
    out[1] = msg.linear.y;
    out[2] = msg.linear.z;
    out[3] = msg.angular.x;
    out[4] = msg.angular.y;
    out[5] = msg.angular.z;
  }
};

template <>
struct defaultMessage<Eigen::Matrix<double, 6, 1>>
{
  using type = geometry_msgs::Twist;
};

}  // namespace impl

/** \brief Apply a geometry_msgs TransformStamped to an Eigen Affine3d transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * function relies on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The frame to transform, as a timestamped Eigen Affine3d transform.
 * \param t_out The transformed frame, as a timestamped Eigen Affine3d transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Affine3d>& t_in,
		 tf2::Stamped<Eigen::Affine3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Affine3d>(transformToEigen(transform) * t_in, transform.header.stamp, transform.header.frame_id);
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen Isometry transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * function relies on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The frame to transform, as a timestamped Eigen Isometry transform.
 * \param t_out The transformed frame, as a timestamped Eigen Isometry transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Isometry3d>& t_in,
		 tf2::Stamped<Eigen::Isometry3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Isometry3d>(transformToEigen(transform) * t_in, transform.header.stamp, transform.header.frame_id);
}


} // namespace

#endif // TF2_EIGEN_H
