
/*
 * Copyright (c) 2013, Open Source Robotics Foundation
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

#ifndef TF2_IMPL_STAMPED_TRAITS_H_
#define TF2_IMPL_STAMPED_TRAITS_H_

// forward declarations
namespace geometry_msgs
{
template <typename Alloc>
class Point_;
template <typename Alloc>
class Vector_;
template <typename Alloc>
class Quaternion_;
template <typename Alloc>
class Pose_;
template <typename Alloc>
class Twist_;
template <typename Alloc>
class PoseWithCovariance_;
template <typename Alloc>
class Wrench_;
template <typename Alloc>
class PointStamped_;
template <typename Alloc>
class VectorStamped_;
template <typename Alloc>
class QuaternionStamped_;
template <typename Alloc>
class PoseStamped_;
template <typename Alloc>
class TwistStamped_;
template <typename Alloc>
class PoseWithCovarianceStamped_;
template <typename Alloc>
class WrenchStamped_;
template <typename Alloc>
class TransformStamped_;
template <typename Alloc>
class Transform_;
template <typename Alloc>
class Vector3_;
template <typename Alloc>
class Vector3Stamped_;
}  // namespace geometry_msgs

namespace tf2
{
namespace impl
{
template <class UnstampedMessage>
struct unstampedMessageTraits;

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Point_<Alloc>>
{
  using stampedType = geometry_msgs::PointStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Vector_<Alloc>>
{
  using stampedType = geometry_msgs::VectorStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Quaternion_<Alloc>>
{
  using stampedType = geometry_msgs::QuaternionStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Pose_<Alloc>>
{
  using stampedType = geometry_msgs::PoseStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Twist_<Alloc>>
{
  using stampedType = geometry_msgs::TwistStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::PoseWithCovariance_<Alloc>>
{
  using stampedType = geometry_msgs::PoseWithCovarianceStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Wrench_<Alloc>>
{
  using stampedType = geometry_msgs::WrenchStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Transform_<Alloc>>
{
  using stampedType = geometry_msgs::TransformStamped_<Alloc>;
};

template <typename Alloc>
struct unstampedMessageTraits<geometry_msgs::Vector3_<Alloc>>
{
  using stampedType = geometry_msgs::Vector3Stamped_<Alloc>;
};

template <class StampedMessage>
struct stampedMessageTraits;

// we use partial specializations (with the allocator as template parameter)
// to avoid including all the message definitons

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::PointStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Point_<Alloc>;
  static geometry_msgs::Point_<Alloc>& accessMessage(geometry_msgs::PointStamped_<Alloc>& smsg)
  {
    return smsg.point;
  }
  static geometry_msgs::Point_<Alloc> getMessage(geometry_msgs::PointStamped_<Alloc> const& smsg)
  {
    return smsg.point;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::VectorStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Vector_<Alloc>;
  static geometry_msgs::Vector_<Alloc>& accessMessage(geometry_msgs::VectorStamped_<Alloc>& smsg)
  {
    return smsg.vector;
  }
  static geometry_msgs::Vector_<Alloc> getMessage(geometry_msgs::VectorStamped_<Alloc> const& smsg)
  {
    return smsg.vector;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::QuaternionStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Quaternion_<Alloc>;
  static geometry_msgs::Quaternion_<Alloc>& accessMessage(geometry_msgs::QuaternionStamped_<Alloc>& smsg)
  {
    return smsg.quaternion;
  }
  static geometry_msgs::Quaternion_<Alloc> getMessage(geometry_msgs::QuaternionStamped_<Alloc> const& smsg)
  {
    return smsg.quaternion;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::PoseStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Pose_<Alloc>;
  static geometry_msgs::Pose_<Alloc>& accessMessage(geometry_msgs::PoseStamped_<Alloc>& smsg)
  {
    return smsg.pose;
  }
  static geometry_msgs::Pose_<Alloc> getMessage(geometry_msgs::PoseStamped_<Alloc> const& smsg)
  {
    return smsg.pose;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::TwistStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Twist_<Alloc>;
  static geometry_msgs::Twist_<Alloc>& accessMessage(geometry_msgs::TwistStamped_<Alloc>& smsg)
  {
    return smsg.twist;
  }
  static geometry_msgs::Twist_<Alloc> getMessage(geometry_msgs::TwistStamped_<Alloc> const& smsg)
  {
    return smsg.twist;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::PoseWithCovarianceStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::PoseWithCovariance_<Alloc>;
  static geometry_msgs::PoseWithCovariance_<Alloc>&
  accessMessage(geometry_msgs::PoseWithCovarianceStamped_<Alloc>& smsg)
  {
    return smsg.pose;
  }
  static geometry_msgs::PoseWithCovariance_<Alloc>
  getMessage(geometry_msgs::PoseWithCovarianceStamped_<Alloc> const& smsg)
  {
    return smsg.pose;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::WrenchStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Wrench_<Alloc>;
  static geometry_msgs::Wrench_<Alloc>& accessMessage(geometry_msgs::WrenchStamped_<Alloc>& smsg)
  {
    return smsg.wrench;
  }
  static geometry_msgs::Wrench_<Alloc> getMessage(geometry_msgs::WrenchStamped_<Alloc> const& smsg)
  {
    return smsg.wrench;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::TransformStamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Transform_<Alloc>;
  static geometry_msgs::Transform_<Alloc>& accessMessage(geometry_msgs::TransformStamped_<Alloc>& smsg)
  {
    return smsg.transform;
  }

  static geometry_msgs::Transform_<Alloc> getMessage(geometry_msgs::TransformStamped_<Alloc> const& smsg)
  {
    return smsg.transform;
  }
};

template <typename Alloc>
struct stampedMessageTraits<geometry_msgs::Vector3Stamped_<Alloc>>
{
  using unstampedType = geometry_msgs::Vector3_<Alloc>;
  static geometry_msgs::Vector3_<Alloc>& accessMessage(geometry_msgs::Vector3Stamped_<Alloc>& smsg)
  {
    return smsg.vector;
  }

  static geometry_msgs::Vector3_<Alloc> getMessage(geometry_msgs::Vector3Stamped_<Alloc> const& smsg)
  {
    return smsg.vector;
  }
};

}  // namespace impl
}  // namespace tf2

#endif  // TF2_IMPL_STAMPED_TRAITS_H_
