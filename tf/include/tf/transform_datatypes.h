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

/** \author Tully Foote */

#ifndef TF_TRANSFORM_DATATYPES_H
#define TF_TRANSFORM_DATATYPES_H

#include <string>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "LinearMath/btTransform.h"
#include "ros/time.h"

#include "ros/console.h"

namespace tf
{
/** \brief A representaton of orientation or rotation depending on context*/
typedef btQuaternion Quaternion; ///\todo differentiate?
/** \brief A representation of a translation */
typedef btVector3 Vector3;
/** \brief  The transform library representation of a point(Position)*/
typedef btVector3 Point;
/** \brief A representation of a translation and rotation */
typedef btTransform Transform;
/** \brief A representation of pose (A position and orientation)*/
typedef btTransform Pose;

static const double QUATERNION_TOLERANCE = 0.1f;

/** \brief The data type which will be cross compatable with geometry_msgs
 * this will require the associated rosTF package to convert */
template <typename T>
class Stamped : public T{
 public:
  ros::Time stamp_;
  std::string frame_id_;
  std::string parent_id_; ///only used for transform

  Stamped() :frame_id_ ("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"), parent_id_("NOT A TRANSFORM"){}; //Default constructor used only for preallocation

  Stamped(const T& input, const ros::Time& timestamp, const std::string & frame_id, const std::string & parent_id = "NOT A TRANSFORM"):
    T (input), stamp_ ( timestamp ), frame_id_ (frame_id), parent_id_(parent_id){ };

//Stamped(const Stamped<T>& input):data_(input.data_), stamp_(input.stamp_), frame_id_(input.frame_id_), parent_id_(input.parent_id_){};

//Stamped& operator=(const Stamped<T>& input){data_ = input.data_; stamp_ = input.stamp_; frame_id_ = input.frame_id_;
//  parent_id_ = input.parent_id_; return *this;};

  void setData(const T& input){*static_cast<T*>(this) = input;};
  //  void stripStamp(T & output) { output = data_;}; //just down cast it
};


/** \brief convert Quaternion msg to Quaternion */
static inline void quaternionMsgToTF(const geometry_msgs::Quaternion& msg, Quaternion& bt) 
{
  bt = Quaternion(msg.x, msg.y, msg.z, msg.w); 
  if (fabs(bt.length2() - 1 ) > QUATERNION_TOLERANCE) 
    {
      ROS_WARN("MSG to TF: Quaternion Not Properly Normalized");
      bt.normalize();
    }
};
/** \brief convert Quaternion to Quaternion msg*/
static inline void quaternionTFToMsg(const Quaternion& bt, geometry_msgs::Quaternion& msg) 
{
  if (fabs(bt.length2() - 1 ) > QUATERNION_TOLERANCE) 
    {
      ROS_WARN("TF to MSG: Quaternion Not Properly Normalized");
      Quaternion bt_temp = bt; 
      bt_temp.normalize();
      msg.x = bt_temp.x(); msg.y = bt_temp.y(); msg.z = bt_temp.z();  msg.w = bt_temp.w();
    }
  else
  {
    msg.x = bt.x(); msg.y = bt.y(); msg.z = bt.z();  msg.w = bt.w();
  }
};

/** \brief Helper function for getting yaw from a Quaternion */
static inline double getYaw(const Quaternion& bt_q){
  btScalar useless_pitch, useless_roll, yaw;
  btMatrix3x3(bt_q).getEulerZYX(yaw, useless_pitch, useless_roll);
  return yaw;
}

/** \brief Helper function for getting yaw from a Quaternion message*/
static inline double getYaw(const geometry_msgs::Quaternion& msg_q){
  Quaternion bt_q;
  quaternionMsgToTF(msg_q, bt_q);
  return getYaw(bt_q);
}

static inline Quaternion createQuaternionFromYaw(double yaw){
  return Quaternion(yaw, 0.0, 0.0);
}

static inline geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw){
  Quaternion q(yaw, 0.0, 0.0);
  geometry_msgs::Quaternion q_msg;
  quaternionTFToMsg(q, q_msg);
  return q_msg;
}

/** \brief convert QuaternionStamped msg to Stamped<Quaternion> */
static inline void quaternionStampedMsgToTF(const geometry_msgs::QuaternionStamped & msg, Stamped<Quaternion>& bt)
{quaternionMsgToTF(msg.quaternion, bt); bt.stamp_ = msg.header.stamp; bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Quaternion> to QuaternionStamped msg*/
static inline void quaternionStampedTFToMsg(const Stamped<Quaternion>& bt, geometry_msgs::QuaternionStamped & msg)
{quaternionTFToMsg(bt, msg.quaternion); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};

/** \brief convert Vector3 msg to Vector3 */
static inline void vector3MsgToTF(const geometry_msgs::Vector3& msg_v, Vector3& bt_v) {bt_v = Vector3(msg_v.x, msg_v.y, msg_v.z);};
/** \brief convert Vector3 to Vector3 msg*/
static inline void vector3TFToMsg(const Vector3& bt_v, geometry_msgs::Vector3& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};

/** \brief convert Vector3Stamped msg to Stamped<Vector3> */
static inline void vector3StampedMsgToTF(const geometry_msgs::Vector3Stamped & msg, Stamped<Vector3>& bt)
{vector3MsgToTF(msg.vector, bt); bt.stamp_ = msg.header.stamp; bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Vector3> to Vector3Stamped msg*/
static inline void vector3StampedTFToMsg(const Stamped<Vector3>& bt, geometry_msgs::Vector3Stamped & msg)
{vector3TFToMsg(bt, msg.vector); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};


/** \brief convert Point msg to Point */
static inline void pointMsgToTF(const geometry_msgs::Point& msg_v, Point& bt_v) {bt_v = Vector3(msg_v.x, msg_v.y, msg_v.z);};
/** \brief convert Point to Point msg*/
static inline void pointTFToMsg(const Point& bt_v, geometry_msgs::Point& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};

/** \brief convert PointStamped msg to Stamped<Point> */
static inline void pointStampedMsgToTF(const geometry_msgs::PointStamped & msg, Stamped<Point>& bt)
{pointMsgToTF(msg.point, bt); bt.stamp_ = msg.header.stamp; bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Point> to PointStamped msg*/
static inline void pointStampedTFToMsg(const Stamped<Point>& bt, geometry_msgs::PointStamped & msg)
{pointTFToMsg(bt, msg.point); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};


/** \brief convert Transform msg to Transform */
static inline void transformMsgToTF(const geometry_msgs::Transform& msg, Transform& bt)
{bt = Transform(Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), Vector3(msg.translation.x, msg.translation.y, msg.translation.z));};
/** \brief convert Transform to Transform msg*/
static inline void transformTFToMsg(const Transform& bt, geometry_msgs::Transform& msg)
{vector3TFToMsg(bt.getOrigin(), msg.translation);  quaternionTFToMsg(bt.getRotation(), msg.rotation);};

/** \brief convert TransformStamped msg to Stamped<Transform> */
static inline void transformStampedMsgToTF(const geometry_msgs::TransformStamped & msg, Stamped<Transform>& bt)
{transformMsgToTF(msg.transform, bt); bt.stamp_ = msg.header.stamp; bt.parent_id_ = msg.header.frame_id; bt.frame_id_ = msg.child_frame_id;};
/** \brief convert Stamped<Transform> to TransformStamped msg*/
static inline void transformStampedTFToMsg(const Stamped<Transform>& bt, geometry_msgs::TransformStamped & msg)
{transformTFToMsg(bt, msg.transform); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.parent_id_; msg.child_frame_id = bt.frame_id_;};

/** \brief convert Pose msg to Pose */
static inline void poseMsgToTF(const geometry_msgs::Pose& msg, Pose& bt)
{bt = Transform(Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), Vector3(msg.position.x, msg.position.y, msg.position.z));};
/** \brief convert Pose to Pose msg*/
static inline void poseTFToMsg(const Pose& bt, geometry_msgs::Pose& msg)
{pointTFToMsg(bt.getOrigin(), msg.position);  quaternionTFToMsg(bt.getRotation(), msg.orientation);};

/** \brief convert PoseStamped msg to Stamped<Pose> */
static inline void poseStampedMsgToTF(const geometry_msgs::PoseStamped & msg, Stamped<Pose>& bt)
{poseMsgToTF(msg.pose, bt); bt.stamp_ = msg.header.stamp; bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Pose> to PoseStamped msg*/
static inline void poseStampedTFToMsg(const Stamped<Pose>& bt, geometry_msgs::PoseStamped & msg)
{poseTFToMsg(bt, msg.pose); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};






}
#endif //TF_TRANSFORM_DATATYPES_H
