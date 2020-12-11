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

/** \author Tully Foote */

#ifndef TF2_CONVERT_H
#define TF2_CONVERT_H

#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/TransformStamped.h>

namespace tf2
{
namespace impl
{
/**
 * \brief Mapping between Datatypes (like \c Vector3d ) and their default ROS Message types.
 *
 * This struct should be specialized for each non-Message datatypes,
 * and it should contain an alias of the Message class with the name \c type .
 * This alias will be used to deduce the return value of tf2::toMsg().
 *
 * \tparam Datatype Non-Message datatype like \c Vector3d
 */
template <class Datatype, class = void>
struct defaultMessage
{
  // using type = ...;
};

/**
 * \brief Conversion details between a Message and a non-Message datatype.
 * \tparam Datatype Non-Message datatype like \c Vector3d
 * \tparam Message  The ROS Message class
 *
 * The specializations of this struct should contain two static methods,
 * which convert a ROS Message into the requested datatype and vice versa.
 * They should have the following signature:
 * \code
 * template<>
 * struct defautMessage<Datatype, Message>
 * {
 *   static void toMsg(const Datatype&, Message&);
 *   static void fromMsg(const Message&, Datatype&);
 * }:
 * \endcode
 * Note that the conversion between tf2::Stamped\<Datatype\> and
 * geometry_msgs::...Stamped is done automatically.
 */
template <class Datatype, class Message, class = void>
struct ImplDetails
{
  // void toMsg(const Datatype&, Message&);
  // void fromMsg(const Message&, Datatype&);
};

// Forward declaration for the extraction of timestamps and frame IDs
template <typename T, int = 0>
struct DefaultStampedImpl;
// Forward declaration for the tf2::convert() implementation
template <bool, bool>
class Converter;

}  // namespace impl

/**\brief The templated function expected to be able to do a transform
 *
 * This is the method which tf2 will use to try to apply a transform for any given datatype.
 * \param data_in The data to be transformed.
 * \param data_out A reference to the output data.  Note this can point to data in and the method should be mutation
 * safe. \param transform The transform to apply to data_in to fill data_out.
 *
 * This method needs to be implemented by client library developers
 */
template <class T>
void doTransform(const T& data_in, T& data_out, const geometry_msgs::TransformStamped& transform);

/**\brief Get the timestamp from data
 * \param t The data input.
 * \tparam T The type of the data input.
 * \return The timestamp associated with the data. The lifetime of the returned
 * reference is bound to the lifetime of the argument.
 *
 * Library developers need to check whether the default implementation
 * is sufficient, extend the default implementation
 * or provide a specialization of this function.
 */
template <class T>
inline
const ros::Time& getTimestamp(const T& t)
{
  return impl::DefaultStampedImpl<T>::getTimestamp(t);
}

/**\brief Get the frame_id from data
 * \param t The data input.
 * \tparam T The type of the data input.
 * \return The frame_id associated with the data. The lifetime of the returned
 * reference is bound to the lifetime of the argument.
 *
 * Library developers need to check whether the default implementation
 * is sufficient, extend the default implementation
 * or provide a specialization of this function.
 */
template <class T>
inline
const std::string& getFrameId(const T& t)
{
  return impl::DefaultStampedImpl<T>::getFrameId(t);
}

/**
 * \brief Function that converts from one type to a ROS message type.
 *
 * The implementation of this function should be done in the tf2_* packages
 * for each datatypes. Preferably in a specialization of the impl::ImplDetails struct.
 * \param a an object of whatever type
 * \tparam A Non-message Datatype
 * \tparam B ROS message Datatype. The default value will be taken from impl::defaultMessage\<A\>::type.
 * \return the conversion as a ROS message
 */
template <typename A, typename B = typename impl::defaultMessage<A>::type>
inline B toMsg(const A& a)
{
  B b;
  impl::ImplDetails<A, B>::toMsg(a, b);
  return b;
}


/**
 * \brief Function that converts from one type to a ROS message type.
 *
 * The implementation of this function should be done in the tf2_* packages
 * for each datatypes. Preferably in a specialization of the impl::ImplDetails struct.
 * \param a an object of whatever type
 * \param b ROS message
 * \tparam A Non-message Datatype
 * \tparam B Type of the ROS Message
 * \return Reference to the parameter b
 */
template <typename A, typename B>
inline B& toMsg(const A& a, B& b)
{
  impl::ImplDetails<A, B>::toMsg(a, b);
  return b;
}

/**
 * \brief Function that converts from a ROS message type to another type.
 *
 * The implementation of this function should be done in the tf2_* packages
 * for each datatypes. Preferably in a specialization of the impl::ImplDetails struct.
 * \param a a ROS message to convert from
 * \param b the object to convert to
 * \tparam A ROS message type
 * \tparam B Arbitrary type
 */
template <typename A, typename B>
inline void fromMsg(const A& a, B& b)
{
  impl::ImplDetails<B, A>::fromMsg(a, b);
}

/**
 * \brief Function that converts any type to any type (messages or not).
 *
 * Matching toMsg and from Msg conversion functions need to exist.
 * If they don't exist or do not apply (for example, if your two
 * classes are ROS messages), just write a specialization of the function.
 * \param a an object to convert from
 * \param b the object to convert to
 * \tparam A Type of the object to convert from
 * \tparam B Type of the object to convert to
 */
template <class A, class B>
inline void convert(const A& a, B& b)
{
  impl::Converter<ros::message_traits::IsMessage<A>::value, ros::message_traits::IsMessage<B>::value>::convert(a, b);
}

template <class A>
inline void convert(const A& a1, A& a2)
{
  if (&a1 != &a2)
    a2 = a1;
}

}  // namespace tf2

#include <tf2/impl/convert.h>
#include <tf2/impl/stamped_traits.h>

#endif  // TF2_CONVERT_H
