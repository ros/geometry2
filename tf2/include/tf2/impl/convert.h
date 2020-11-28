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

#ifndef TF2_IMPL_CONVERT_H
#define TF2_IMPL_CONVERT_H

namespace tf2
{
namespace impl
{

template <class StampedMessage>
struct stampedMessageTraits
{
  // using unstampedType = ...;
  // static unstampedType& accessMessage(StampedMsg &);
  // static unstampedType getMessage(StampedMsg const&);
};

template <class UnstampedMessage>
struct unstampedMessageTraits
{
  // using stampedType = ...;
};

template <class T>
struct defaultMessage<tf2::Stamped<T>>
{
  using type = typename unstampedMessageTraits<typename defaultMessage<T>::type>::stampedType;
};


template <class Datatype, class StampedMessage>
struct ImplDetails<tf2::Stamped<Datatype>, StampedMessage>
{
  using traits = stampedMessageTraits<StampedMessage>;
  using unstampedMessage = typename traits::unstampedType;

  static void toMsg(const tf2::Stamped<Datatype>& s, StampedMessage& msg)
  {
    tf2::toMsg<>(static_cast<const Datatype&>(s), traits::accessMessage(msg));
    msg.header.stamp = s.stamp_;
    msg.header.frame_id = s.frame_id_;
  }

  static void fromMsg(const StampedMessage& msg, tf2::Stamped<Datatype>& s)
  {
    tf2::fromMsg<>(traits::getMessage(msg), static_cast<Datatype&>(s));
    s.stamp_ = msg.header.stamp;
    s.frame_id_ = msg.header.frame_id;
  }
};

template <bool IS_MESSAGE_A, bool IS_MESSAGE_B>
class Converter
{
public:
  template <typename A, typename B>
  static void convert(const A& a, B& b);
};

// The case where both A and B are messages should not happen: if you have two
// messages that are interchangeable, well, that's against the ROS purpose:
// only use one type. Worst comes to worst, specialize the original convert
// function for your types.
// if B == A, the templated version of convert with only one argument will be
// used.
//
// template <>
// template <typename A, typename B>
// inline void Converter<true, true>::convert(const A& a, B& b);

template <>
template <typename A, typename B>
inline void Converter<true, false>::convert(const A& a, B& b)
{
  tf2::fromMsg<>(a, b);
}

template <>
template <typename A, typename B>
inline void Converter<false, true>::convert(const A& a, B& b)
{
  b = tf2::toMsg<>(a);
}

template <>
template <typename A, typename B>
inline void Converter<false, false>::convert(const A& a, B& b)
{
  tf2::fromMsg<>(tf2::toMsg<>(a), b);
}

template <typename T>
using void_t = void;

template <typename T, int>
struct DefaultStampedImpl
{
  static const ros::Time& getTimestamp(const T& t, void_t<typename stampedMessageTraits<T>::unstampedType>* = nullptr)
  {
    return t.header.stamp;
  }

  static const std::string& getFrameId(const T& t, void_t<typename stampedMessageTraits<T>::unstampedType>* = nullptr)
  {
    return t.header.frame_id;
  }
};

template <typename T>
struct DefaultStampedImpl<tf2::Stamped<T>>
{
  static const ros::Time& getTimestamp(const tf2::Stamped<T>& t)
  {
    return t.stamp_;
  }
  static const std::string& getFrameId(const tf2::Stamped<T>& t)
  {
    return t.frame_id_;
  }
};


}  // namespace impl
}  // namespace tf2

#endif  // TF2_IMPL_CONVERT_H
