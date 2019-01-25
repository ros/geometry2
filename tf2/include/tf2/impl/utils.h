// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TF2_IMPL_UTILS_H
#define TF2_IMPL_UTILS_H

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

namespace tf2 {
namespace impl {

// functions moved to tf2_geometry_msgs/tf2_geometry_msgs.h to break a cycle
#ifndef TF2_GEOMETRY_MSGS_H
inline
tf2::Quaternion toQuaternion(const tf2::Quaternion& q)
__attribute__((deprecated("toQuaternion moved to tf2_geometry_msgs, please include tf2_geometry_msgs/tf2_geometry_msgs.h before tf2/impl/utils.h")));

template<typename T>
  tf2::Quaternion toQuaternion(const tf2::Stamped<T>& t)
__attribute__((deprecated("toQuaternion moved to tf2_geometry_msgs, please include tf2_geometry_msgs/tf2_geometry_msgs.h before tf2/impl/utils.h")));

template<typename T>
  tf2::Quaternion toQuaternion(const T& t)
__attribute__((deprecated("toQuaternion moved to tf2_geometry_msgs, please include tf2_geometry_msgs/tf2_geometry_msgs.h before tf2/impl/utils.h")));
#endif

/** The code below is blantantly copied from urdfdom_headers
 * only the normalization has been added.
 * It computes the Euler roll, pitch yaw from a tf2::Quaternion
 * It is equivalent to tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
 * \param q a tf2::Quaternion
 * \param yaw the computed yaw
 * \param pitch the computed pitch
 * \param roll the computed roll
 */
inline
void getEulerYPR(const tf2::Quaternion& q, double &yaw, double &pitch, double &roll)
{
  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x()*q.z() - q.w()*q.y()) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
  if (sarg <= -0.99999) {
    pitch = -0.5*M_PI;
    roll  = 0;
    yaw   = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    pitch = 0.5*M_PI;
    roll  = 0;
    yaw   = 2 * atan2(q.y(), q.x());
  } else {
    pitch = asin(sarg);
    roll  = atan2(2 * (q.y()*q.z() + q.w()*q.x()), sqw - sqx - sqy + sqz);
    yaw   = atan2(2 * (q.x()*q.y() + q.w()*q.z()), sqw + sqx - sqy - sqz);
  }
};

/** The code below is a simplified version of getEulerRPY that only
 * returns the yaw. It is mostly useful in navigation where only yaw
 * matters
 * \param q a tf2::Quaternion
 * \return the computed yaw
 */
inline
double getYaw(const tf2::Quaternion& q)
{
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x()*q.z() - q.w()*q.y()) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

  if (sarg <= -0.99999) {
    yaw   = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    yaw   = 2 * atan2(q.y(), q.x());
  } else {
    yaw   = atan2(2 * (q.x()*q.y() + q.w()*q.z()), sqw + sqx - sqy - sqz);
  }
  return yaw;
};

}
}

#endif //TF2_IMPL_UTILS_H
