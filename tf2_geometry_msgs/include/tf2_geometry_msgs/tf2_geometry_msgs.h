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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl/frames.hpp>

#include <array>

#include "ros/macros.h"

namespace tf2
{

/** \brief Convert a TransformStamped message to a KDL frame.
 * \param t TransformStamped message to convert.
 * \return The converted KDL Frame.
 * \deprecated
 */
inline
ROS_DEPRECATED KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t);
inline
KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y, 
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }

namespace impl
{
/*************/
/** Vector3 **/
/*************/

template <typename Msg>
struct tf2VectorImplDetails
{
  /** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in A tf2 Vector3 object.
   * \return The Vector3 converted to a geometry_msgs message type.
   */
  static void toMsg(const tf2::Vector3& in, Msg& out)
  {
    out.x = in.getX();
    out.y = in.getY();
    out.z = in.getZ();
  }

  /** \brief Convert a Vector3 message to its equivalent tf2 representation.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h.
   * \param in A Vector3 message type.
   * \param out The Vector3 converted to a tf2 type.
   */
  static void fromMsg(const Msg& in, tf2::Vector3& out)
  {
    out = tf2::Vector3(in.x, in.y, in.z);
  }
};

template <>
struct ImplDetails<tf2::Vector3, geometry_msgs::Vector3> : tf2VectorImplDetails<geometry_msgs::Vector3>
{
};

template <>
struct ImplDetails<tf2::Vector3, geometry_msgs::Point> : tf2VectorImplDetails<geometry_msgs::Point>
{
};

template <>
struct defaultMessage<tf2::Vector3>
{
  using type = geometry_msgs::Vector3;
};

/****************/
/** Quaternion **/
/****************/

template <>
struct ImplDetails<tf2::Quaternion, geometry_msgs::Quaternion>
{
  /** \brief Convert a tf2 Quaternion type to its equivalent geometry_msgs representation.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in A tf2 Quaternion object.
   * \return The Quaternion converted to a geometry_msgs message type.
   */
  static void toMsg(const tf2::Quaternion& in, geometry_msgs::Quaternion& out)
  {
    out.w = in.getW();
    out.x = in.getX();
    out.y = in.getY();
    out.z = in.getZ();
  }

  /** \brief Convert a Quaternion message to its equivalent tf2 representation.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h.
   * \param in A Quaternion message type.
   * \param out The Quaternion converted to a tf2 type.
   */
  static void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
  {
    // w at the end in the constructor
    out = tf2::Quaternion(in.x, in.y, in.z, in.w);
  }
};

template <>
struct defaultMessage<tf2::Quaternion>
{
  using type = geometry_msgs::Quaternion;
};

/**********/
/** Pose **/
/**********/

template <>
struct ImplDetails<tf2::Transform, geometry_msgs::Pose>
{
  /** \brief Convert a tf2 Transform type to an equivalent geometry_msgs Pose message.
   * \param in A tf2 Transform object.
   * \param out The Transform converted to a geometry_msgs Pose message type.
   */
  static void toMsg(const tf2::Transform& in, geometry_msgs::Pose& out)
  {
    tf2::toMsg(in.getOrigin(), out.position);
    tf2::toMsg(in.getRotation(), out.orientation);
  }

  /** \brief Convert a geometry_msgs Pose message to an equivalent tf2 Transform type.
   * \param in A Pose message.
   * \param out The Pose converted to a tf2 Transform type.
   */
  static void fromMsg(const geometry_msgs::Pose& in, tf2::Transform& out)
  {
    out.setOrigin(tf2::Vector3(in.position.x, in.position.y, in.position.z));
    // w at the end in the constructor
    out.setRotation(tf2::Quaternion(in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w));
  }
};

/*******************************/
/** PoseWithCovarianceStamped **/
/*******************************/

template <>
struct ImplDetails<tf2::Transform, geometry_msgs::PoseWithCovariance>
{
  /** \brief Convert a PoseWithCovarianceStamped message to its equivalent tf2 representation.
   * This function is a specialization of the fromMsg template defined in tf2/convert.h.
   * \param msg A PoseWithCovarianceStamped message.
   * \param out The PoseWithCovarianceStamped converted to the equivalent tf2 type.
   */
  static void fromMsg(const geometry_msgs::PoseWithCovariance& msg, tf2::Transform& out)
  {
    tf2::fromMsg<>(msg.pose, out);
  }
};

/***************/
/** Transform **/
/***************/

template <>
struct ImplDetails<tf2::Transform, geometry_msgs::Transform>
{
  /** \brief Convert a tf2 Transform type to its equivalent geometry_msgs representation.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in A tf2 Transform object.
   * \return The Transform converted to a geometry_msgs message type.
   */
  static void toMsg(const tf2::Transform& in, geometry_msgs::Transform& out)
  {
    tf2::toMsg(in.getOrigin(), out.translation);
    tf2::toMsg(in.getRotation(), out.rotation);
  }

  /** \brief Convert a Transform message to its equivalent tf2 representation.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param in A Transform message type.
   * \param out The Transform converted to a tf2 type.
   */
  static void fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out)
  {
    tf2::Vector3 v;
    tf2::fromMsg(in.translation, v);
    out.setOrigin(v);
    // w at the end in the constructor
    tf2::Quaternion q;
    tf2::fromMsg(in.rotation, q);
    out.setRotation(q);
  }
};

template <>
struct defaultMessage<tf2::Transform>
{
  using type = geometry_msgs::Transform;
};
}  // namespace impl

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point3 message.
 * \param t_out The transformed point, as a Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Point& t_in, geometry_msgs::Point& t_out, const geometry_msgs::TransformStamped& transform)
  {
  tf2::Transform t;
  tf2::fromMsg<>(transform.transform, t);
  tf2::Vector3 v_in;
  tf2::fromMsg<>(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  tf2::toMsg<>(v_out, t_out);
  }

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a timestamped Point3 message.
 * \param t_out The transformed point, as a timestamped Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::PointStamped& t_in, geometry_msgs::PointStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    doTransform(t_in.point, t_out.point, transform);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The quaternion to transform, as a Quaternion3 message.
 * \param t_out The transformed quaternion, as a Quaternion3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::Quaternion& t_in, geometry_msgs::Quaternion& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Quaternion t, q_in;
  tf2::fromMsg<>(transform.transform.rotation, t);
  tf2::fromMsg<>(t_in, q_in);

  tf2::Quaternion q_out = t * q_in;
  tf2::toMsg<>(q_out, t_out);
}

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The quaternion to transform, as a timestamped Quaternion3 message.
 * \param t_out The transformed quaternion, as a timestamped Quaternion3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::QuaternionStamped& t_in, geometry_msgs::QuaternionStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.quaternion, t_out.quaternion, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}


/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a Pose3 message.
* \param t_out The transformed pose, as a Pose3 message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::Pose& t_in, geometry_msgs::Pose& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  tf2::fromMsg<>(t_in.position, v);
  tf2::Quaternion r;
  tf2::fromMsg<>(t_in.orientation, r);

  tf2::Transform t;
  tf2::fromMsg<>(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  tf2::toMsg<>(v_out, t_out);
}

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Pose type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a timestamped Pose3 message.
* \param t_out The transformed pose, as a timestamped Pose3 message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::PoseStamped& t_in, geometry_msgs::PoseStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.pose, t_out.pose, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/** \brief Transform the covariance matrix of a PoseWithCovarianceStamped message to a new frame.
* \param t_in The covariance matrix to transform.
* \param transform The timestamped transform to apply, as a TransformStamped message.
* \return The transformed covariance matrix.
*/
inline
geometry_msgs::PoseWithCovariance::_covariance_type transformCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& cov_in, const tf2::Transform& transform)
{
  /**
   * To transform a covariance matrix:
   * 
   * [R 0] COVARIANCE [R' 0 ]
   * [0 R]            [0  R']
   * 
   * Where:
   * 	R is the rotation matrix (3x3).
   * 	R' is the transpose of the rotation matrix.
   * 	COVARIANCE is the 6x6 covariance matrix to be transformed.
   */ 
  
  // get rotation matrix transpose  
  const tf2::Matrix3x3  R_transpose = transform.getBasis().transpose();
  
  // convert the covariance matrix into four 3x3 blocks
  const tf2::Matrix3x3 cov_11(cov_in[0], cov_in[1], cov_in[2],
			      cov_in[6], cov_in[7], cov_in[8],
			      cov_in[12], cov_in[13], cov_in[14]);
  const tf2::Matrix3x3 cov_12(cov_in[3], cov_in[4], cov_in[5],
			      cov_in[9], cov_in[10], cov_in[11],
			      cov_in[15], cov_in[16], cov_in[17]);
  const tf2::Matrix3x3 cov_21(cov_in[18], cov_in[19], cov_in[20],
			      cov_in[24], cov_in[25], cov_in[26],
			      cov_in[30], cov_in[31], cov_in[32]);
  const tf2::Matrix3x3 cov_22(cov_in[21], cov_in[22], cov_in[23],
			      cov_in[27], cov_in[28], cov_in[29],
			      cov_in[33], cov_in[34], cov_in[35]);
  
  // perform blockwise matrix multiplication
  const tf2::Matrix3x3 result_11 = transform.getBasis()*cov_11*R_transpose;
  const tf2::Matrix3x3 result_12 = transform.getBasis()*cov_12*R_transpose;
  const tf2::Matrix3x3 result_21 = transform.getBasis()*cov_21*R_transpose;
  const tf2::Matrix3x3 result_22 = transform.getBasis()*cov_22*R_transpose;
  
  // form the output
  geometry_msgs::PoseWithCovariance::_covariance_type output;
  output[0] = result_11[0][0];
  output[1] = result_11[0][1];
  output[2] = result_11[0][2];
  output[6] = result_11[1][0];
  output[7] = result_11[1][1];
  output[8] = result_11[1][2];
  output[12] = result_11[2][0];
  output[13] = result_11[2][1];
  output[14] = result_11[2][2];
  
  output[3] = result_12[0][0];
  output[4] = result_12[0][1];
  output[5] = result_12[0][2];
  output[9] = result_12[1][0];
  output[10] = result_12[1][1];
  output[11] = result_12[1][2];
  output[15] = result_12[2][0];
  output[16] = result_12[2][1];
  output[17] = result_12[2][2];
  
  output[18] = result_21[0][0];
  output[19] = result_21[0][1];
  output[20] = result_21[0][2];
  output[24] = result_21[1][0];
  output[25] = result_21[1][1];
  output[26] = result_21[1][2];
  output[30] = result_21[2][0];
  output[31] = result_21[2][1];
  output[32] = result_21[2][2];
  
  output[21] = result_22[0][0];
  output[22] = result_22[0][1];
  output[23] = result_22[0][2];
  output[27] = result_22[1][0];
  output[28] = result_22[1][1];
  output[29] = result_22[1][2];
  output[33] = result_22[2][0];
  output[34] = result_22[2][1];
  output[35] = result_22[2][2];
  
  return output;
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs PoseWithCovarianceStamped type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a timestamped PoseWithCovarianceStamped message.
* \param t_out The transformed pose, as a timestamped PoseWithCovarianceStamped message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::PoseWithCovarianceStamped& t_in, geometry_msgs::PoseWithCovarianceStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  tf2::fromMsg<>(t_in.pose.pose.position, v);
  tf2::Quaternion r;
  tf2::fromMsg<>(t_in.pose.pose.orientation, r);

  tf2::Transform t;
  tf2::fromMsg<>(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  tf2::toMsg<>(v_out, t_out.pose.pose);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;

  t_out.pose.covariance = transformCovariance(t_in.pose.covariance, t);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Transform type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The frame to transform, as a timestamped Transform3 message.
 * \param t_out The frame transform, as a timestamped Transform3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::TransformStamped& t_in, geometry_msgs::TransformStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform input;
  tf2::fromMsg<>(t_in.transform, input);

    tf2::Transform t;
  tf2::fromMsg<>(transform.transform, t);
    tf2::Transform v_out = t * input;

    t_out.transform = toMsg(v_out);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a Vector3 message.
 * \param t_out The transformed vector, as a Vector3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Vector3& t_in, geometry_msgs::Vector3& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform t;
  tf2::fromMsg<>(transform.transform, t);
    tf2::Vector3 v_out = t.getBasis() * tf2::Vector3(t_in.x, t_in.y, t_in.z);
    t_out.x = v_out[0];
    t_out.y = v_out[1];
    t_out.z = v_out[2];
  }

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Vector3 message.
 * \param t_out The transformed vector, as a timestamped Vector3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Vector3Stamped& t_in, geometry_msgs::Vector3Stamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    doTransform(t_in.vector, t_out.vector, transform);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }


/**********************/
/*** WrenchStamped ****/
/**********************/

namespace impl
{
template <>
struct ImplDetails<std::array<tf2::Vector3, 2>, geometry_msgs::Wrench>
{
  static void toMsg(const std::array<tf2::Vector3, 2>& in, geometry_msgs::Wrench& out)
{
    tf2::toMsg(std::get<0>(in), out.force);
    tf2::toMsg(std::get<1>(in), out.torque);
}

  static void fromMsg(const geometry_msgs::Wrench& msg, std::array<tf2::Vector3, 2>& out)
{
    tf2::fromMsg<>(msg.force, std::get<0>(out));
    tf2::fromMsg<>(msg.torque, std::get<1>(out));
}
};
}  // namespace impl

template<>
inline
void doTransform(const geometry_msgs::Wrench& t_in, geometry_msgs::Wrench& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.force, t_out.force, transform);
  doTransform(t_in.torque, t_out.torque, transform);
}


template<>
inline
void doTransform(const geometry_msgs::WrenchStamped& t_in, geometry_msgs::WrenchStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.wrench, t_out.wrench, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

} // namespace

#endif // TF2_GEOMETRY_MSGS_H
