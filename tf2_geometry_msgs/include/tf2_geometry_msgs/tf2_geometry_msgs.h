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
#include <kdl/frames.hpp>

namespace tf2
{

/** \brief Convert a TransformStamped message to a KDL frame.
 * \param t TransformStamped message to convert.
 * \return The converted KDL Frame.
 * \deprecated
 */
inline
KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t) __attribute__ ((deprecated));
inline
KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y, 
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }


/*************/
/** Vector3 **/
/*************/

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Vector3 toMsg(const tf2::Vector3& in)
{
  geometry_msgs::Vector3 out;
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Vector3& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}


/********************/
/** Vector3Stamped **/
/********************/

/** \brief Extract a timestamp from the header of a Vector message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t VectorStamped message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::Vector3Stamped& t) {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Vector message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t VectorStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
  const std::string& getFrameId(const geometry_msgs::Vector3Stamped& t) {return t.header.frame_id;}


/** \brief Trivial "conversion" function for Vector3 message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A Vector3Stamped message.
 * \return The input argument.
 */
inline
geometry_msgs::Vector3Stamped toMsg(const geometry_msgs::Vector3Stamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Vector3 message type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A Vector3Stamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::Vector3Stamped& msg, geometry_msgs::Vector3Stamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Vector3 specialization of the tf2::Stamped template.
 * \return The Vector3Stamped converted to a geometry_msgs Vector3Stamped message type.
 */
inline
geometry_msgs::Vector3Stamped toMsg(const tf2::Stamped<tf2::Vector3>& in)
{
  geometry_msgs::Vector3Stamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.vector.x = in.getX();
  out.vector.y = in.getY();
  out.vector.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3Stamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A Vector3Stamped message.
 * \param out The Vector3Stamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Vector3Stamped& msg, tf2::Stamped<tf2::Vector3>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.setData(tf2::Vector3(msg.vector.x, msg.vector.y, msg.vector.z));
}


/***********/
/** Point **/
/***********/

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Point& toMsg(const tf2::Vector3& in, geometry_msgs::Point& out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Point& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}


/******************/
/** PointStamped **/
/******************/

/** \brief Extract a timestamp from the header of a Point message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PointStamped message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::PointStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Point message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PointStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
  const std::string& getFrameId(const geometry_msgs::PointStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Point message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A PointStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::PointStamped toMsg(const geometry_msgs::PointStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Point message type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A PointStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::PointStamped& msg, geometry_msgs::PointStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Vector3 specialization of the tf2::Stamped template.
 * \return The Vector3Stamped converted to a geometry_msgs PointStamped message type.
 */
inline
geometry_msgs::PointStamped toMsg(const tf2::Stamped<tf2::Vector3>& in, geometry_msgs::PointStamped & out)
{
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.point.x = in.getX();
  out.point.y = in.getY();
  out.point.z = in.getZ();
  return out;
}

/** \brief Convert a PointStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A PointStamped message.
 * \param out The PointStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::PointStamped& msg, tf2::Stamped<tf2::Vector3>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.setData(tf2::Vector3(msg.point.x, msg.point.y, msg.point.z));
}


/****************/
/** Quaternion **/
/****************/

/** \brief Convert a tf2 Quaternion type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Quaternion object.
 * \return The Quaternion converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Quaternion toMsg(const tf2::Quaternion& in)
{
  geometry_msgs::Quaternion out;
  out.w = in.getW();
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Quaternion message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Quaternion message type.
 * \param out The Quaternion converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
{
  // w at the end in the constructor
  out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}


/***********************/
/** QuaternionStamped **/
/***********************/

/** \brief Extract a timestamp from the header of a Quaternion message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t QuaternionStamped message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::QuaternionStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Quaternion message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t QuaternionStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
const std::string& getFrameId(const geometry_msgs::QuaternionStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Quaternion message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A QuaternionStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::QuaternionStamped toMsg(const geometry_msgs::QuaternionStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Quaternion message type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A QuaternionStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::QuaternionStamped& msg, geometry_msgs::QuaternionStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Quaternion type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Quaternion specialization of the tf2::Stamped template.
 * \return The QuaternionStamped converted to a geometry_msgs QuaternionStamped message type.
 */
inline
geometry_msgs::QuaternionStamped toMsg(const tf2::Stamped<tf2::Quaternion>& in)
{
  geometry_msgs::QuaternionStamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.quaternion.w = in.getW();
  out.quaternion.x = in.getX();
  out.quaternion.y = in.getY();
  out.quaternion.z = in.getZ();
  return out;
}

template <>
inline
geometry_msgs::QuaternionStamped toMsg(const tf2::Stamped<tf2::Quaternion>& in)  __attribute__ ((deprecated));


//Backwards compatibility remove when forked for Lunar or newer
template <>
inline
geometry_msgs::QuaternionStamped toMsg(const tf2::Stamped<tf2::Quaternion>& in)
{
  return toMsg(in);
}

/** \brief Convert a QuaternionStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A QuaternionStamped message type.
 * \param out The QuaternionStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::QuaternionStamped& in, tf2::Stamped<tf2::Quaternion>& out)
{
  out.stamp_ = in.header.stamp;
  out.frame_id_ = in.header.frame_id;
  tf2::Quaternion tmp;
  fromMsg(in.quaternion, tmp);
  out.setData(tmp);
}

template<>
inline
void fromMsg(const geometry_msgs::QuaternionStamped& in, tf2::Stamped<tf2::Quaternion>& out) __attribute__ ((deprecated));

//Backwards compatibility remove when forked for Lunar or newer
template<>
inline
void fromMsg(const geometry_msgs::QuaternionStamped& in, tf2::Stamped<tf2::Quaternion>& out)
{
    fromMsg(in, out);
}

/**********/
/** Pose **/
/**********/

/** \brief Convert a tf2 Transform type to an equivalent geometry_msgs Pose message.
 * \param in A tf2 Transform object.
 * \param out The Transform converted to a geometry_msgs Pose message type.
 */
inline
geometry_msgs::Pose& toMsg(const tf2::Transform& in, geometry_msgs::Pose& out)
{
  toMsg(in.getOrigin(), out.position);
  out.orientation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a geometry_msgs Pose message to an equivalent tf2 Transform type.
 * \param in A Pose message.
 * \param out The Pose converted to a tf2 Transform type.
 */
inline
void fromMsg(const geometry_msgs::Pose& in, tf2::Transform& out)
{
  out.setOrigin(tf2::Vector3(in.position.x, in.position.y, in.position.z));
  // w at the end in the constructor
  out.setRotation(tf2::Quaternion(in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w));
}


/*****************/
/** PoseStamped **/
/*****************/

/** \brief Extract a timestamp from the header of a Pose message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PoseStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::PoseStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Pose message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PoseStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline
  const std::string& getFrameId(const geometry_msgs::PoseStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A PoseStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::PoseStamped toMsg(const geometry_msgs::PoseStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A PoseStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::PoseStamped& msg, geometry_msgs::PoseStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Pose type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Pose specialization of the tf2::Stamped template.
 * \return The PoseStamped converted to a geometry_msgs PoseStamped message type.
 */
inline
geometry_msgs::PoseStamped toMsg(const tf2::Stamped<tf2::Transform>& in, geometry_msgs::PoseStamped & out)
{
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  toMsg(in.getOrigin(), out.pose.position);
  out.pose.orientation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a PoseStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A PoseStamped message.
 * \param out The PoseStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::PoseStamped& msg, tf2::Stamped<tf2::Transform>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.pose, tmp);
  out.setData(tmp);
}


/***************/
/** Transform **/
/***************/

/** \brief Convert a tf2 Transform type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Transform object.
 * \return The Transform converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Transform toMsg(const tf2::Transform& in)
{
  geometry_msgs::Transform out;
  out.translation = toMsg(in.getOrigin());
  out.rotation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a Transform message to its equivalent tf2 representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A Transform message type.
 * \param out The Transform converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out)
{
  tf2::Vector3 v;
  fromMsg(in.translation, v);
  out.setOrigin(v);
  // w at the end in the constructor
  tf2::Quaternion q;
  fromMsg(in.rotation, q);
  out.setRotation(q);
}


/**********************/
/** TransformStamped **/
/**********************/

/** \brief Extract a timestamp from the header of a Transform message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t TransformStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::TransformStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Transform message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t TransformStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline
const std::string& getFrameId(const geometry_msgs::TransformStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Transform message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A TransformStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::TransformStamped toMsg(const geometry_msgs::TransformStamped& in)
{
  return in;
}

/** \brief Convert a TransformStamped message to its equivalent tf2 representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A TransformStamped message type.
 * \param out The TransformStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::TransformStamped& msg, geometry_msgs::TransformStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Transform type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Transform specialization of the tf2::Stamped template.
 * \return The tf2::Stamped<tf2::Transform> converted to a geometry_msgs TransformStamped message type.
 */
inline
geometry_msgs::TransformStamped toMsg(const tf2::Stamped<tf2::Transform>& in)
{
  geometry_msgs::TransformStamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.transform.translation = toMsg(in.getOrigin());
  out.transform.rotation = toMsg(in.getRotation());
  return out;
}


/** \brief Convert a TransformStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A TransformStamped message.
 * \param out The TransformStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::TransformStamped& msg, tf2::Stamped<tf2::Transform>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.transform, tmp);
  out.setData(tmp);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a timestamped Point3 message.
 * \param t_out The transformed point, as a timestamped Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::PointStamped& t_in, geometry_msgs::PointStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Vector3 v_in;
    fromMsg(t_in.point, v_in);
    tf2::Vector3 v_out = t * v_in;
    toMsg(v_out, t_out.point);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }


/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The quaternion to transform, as a timestamped Quaternion3 message.
 * \param t_out The transformed quaternion, as a timestamped Quaternion3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::QuaternionStamped& t_in, geometry_msgs::QuaternionStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Quaternion t, q_in;
  fromMsg(transform.transform.rotation, t);
  fromMsg(t_in.quaternion, q_in);

  tf2::Quaternion q_out = t * q_in;
  t_out.quaternion = toMsg(q_out);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}


/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a timestamped Pose3 message.
* \param t_out The transformed pose, as a timestamped Pose3 message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::PoseStamped& t_in, geometry_msgs::PoseStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.pose.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.pose.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out.pose);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
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
    fromMsg(t_in.transform, input);

    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Transform v_out = t * input;

    t_out.transform = toMsg(v_out);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Vector3 message.
 * \param t_out The transformed vector, as a timestamped Vector3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Vector3Stamped& t_in, geometry_msgs::Vector3Stamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Vector3 v_out = t.getBasis() * tf2::Vector3(t_in.vector.x, t_in.vector.y, t_in.vector.z);
    t_out.vector.x = v_out[0];
    t_out.vector.y = v_out[1];
    t_out.vector.z = v_out[2];
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }

} // namespace

#endif // TF2_GEOMETRY_MSGS_H
