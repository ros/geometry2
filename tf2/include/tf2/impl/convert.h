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

/**
 * \brief Mapping of unstamped Messages for stamped Messages
 *
 * This struct contains utility methods to access the data member of a stamped ROS message
 * and an alias (named \c unstampedType ) of the unstamped message type.
 * It is needed for the conversion of stamped datatypes,
 * so that only the conversions of unstamped datatypes has do be implemented.
 * For example, a \c geometry_msgs::Vector3Stamped has two members,
 * the \c header (which contains a timestamp and a frame ID) and the \c vector itself.
 * For this class, the specialization should look like
 * \code
 * template<>
 * struct stampedMessageTraits<geometry_msgs::Vector3Stamped>
 * {
 *  using unstampedType = geometry_msgs::Vector3;
 *  static geometry_msgs::Vector3& accessMessage(geometry_msgs::Vector3Stamped& vs)
 *  {
 *     return vs.vector;
 *  }
 *  static geometry_msgs::Vector3 getMessage(const geometry_msgs::Vector3Stamped& vs)
 *  {
 *     return vs.vector;
 *  }
 * };
 * \endcode
 * The both almost identical methods are required to keep const-correctness.
 *
 * \tparam StampedMessage The datatype of the ros message
 */
template <class StampedMessage>
struct stampedMessageTraits
{
  // using unstampedType = ...;
  // static unstampedType& accessMessage(StampedMsg &);
  // static unstampedType getMessage(StampedMsg const&);
};

/**
 * \brief Mapping of stamped Messages for unstamped Messages
 *
 * This struct is needed for the deduction of the return type of
 * tf2::convert() for tf2::Stamped\<\> datatypes.
 * Its specializations should contain an alias (named \c stampedType )
 * of the stamped type.
 * Example:
 * \code
 * template<>
 * struct unstampedMessageTraits<geometry_msgs::Vector3>
 * {
 *    using stampedType = geometry_msgs::Vector3Stamped;
 * };
 * \endcode
 *
 * \tparam UnstampedMessage Type of the ROS message which is not stamped
 */
template <class UnstampedMessage>
struct unstampedMessageTraits
{
  // using stampedType = ...;
};

/**
 * \brief Partial specialization of impl::defaultMessage for stamped types
 *
 * The deduction of the default ROS message type of a tf2::Stamped\<T\> type is
 * based on the default ROS message type of \c T .
 * \tparam T The unstamped datatype (not a ROS message)
 */

template <class T>
struct defaultMessage<tf2::Stamped<T>>
{
  using type = typename unstampedMessageTraits<typename defaultMessage<T>::type>::stampedType;
};

/**
 * \brief Partial specialization of impl::ImplDetails for stamped types
 *
 * This partial specialization provides the conversion implementation ( \c toMsg() and \c fromMsg() )
 * between stamped types ( non-message types of tf2::Stamped\<T\> and ROS message datatypes with a \c header member).
 * The timestamp and the frame ID are preserved during the conversion.
 * The implementation of tf2::toMsg() and tf2::fromMsg() for the unstamped types are required,
 * as well as a specialization of stampedMessageTraits.
 * \tparam Datatype Unstamped non-message type
 * \tparam StampedMessage Stamped ROS message type
 */
template <class Datatype, class StampedMessage>
struct ImplDetails<tf2::Stamped<Datatype>, StampedMessage>
{
  using traits = stampedMessageTraits<StampedMessage>;

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

/**
 * \brief Default implementation for extracting timestamps and frame IDs.
 *
 * Both static member functions are for stamped ROS messages.
 * They are SFINAE'd out if T is not a stamped ROS message.
 *
 * \tparam T Arbitrary datatype
 */
template <typename T, int>
struct DefaultStampedImpl
{
  /**\brief Get the timestamp from data
   * \param t The data input.
   * \return The timestamp associated with the data. The lifetime of the returned
   * reference is bound to the lifetime of the argument.
   *
   * The second parameter is needed to hide the default implementation if T is not a stamped ROS message.
   */
  static const ros::Time& getTimestamp(const T& t, void_t<typename stampedMessageTraits<T>::unstampedType>* = nullptr)
  {
    return t.header.stamp;
  }
  /**\brief Get the frame_id from data
   * \param t The data input.
   * \return The frame_id associated with the data. The lifetime of the returned
   * reference is bound to the lifetime of the argument.
   *
   * The second parameter is needed to hide the default implementation if T is not a stamped ROS message.
   */
  static const std::string& getFrameId(const T& t, void_t<typename stampedMessageTraits<T>::unstampedType>* = nullptr)
  {
    return t.header.frame_id;
  }
};

/**
 * \brief Partial specialization of DefaultStampedImpl for tf2::Stamped\<\> types
 */
template <typename T>
struct DefaultStampedImpl<tf2::Stamped<T>>
{
  /**\brief Get the timestamp from data
   * \param t The data input.
   * \return The timestamp associated with the data. The lifetime of the returned
   * reference is bound to the lifetime of the argument.
   */
  static const ros::Time& getTimestamp(const tf2::Stamped<T>& t)
  {
    return t.stamp_;
  }
  /**  brief Get the frame_id from data
   * \param t The data input.
   * \return The frame_id associated with the data. The lifetime of the returned
   * reference is bound to the lifetime of the argument.
   */
  static const std::string& getFrameId(const tf2::Stamped<T>& t)
  {
    return t.frame_id_;
  }
};


}  // namespace impl
}  // namespace tf2

#endif  // TF2_IMPL_CONVERT_H
