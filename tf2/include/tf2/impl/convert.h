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

#include "cross_convert.h"
#include <type_traits>

namespace tf2 {
namespace impl {

//checks whether (A and B) and (B and A) have no common type
template<class A, class B>
  struct has_no_common_msgs : public std::integral_constant<bool,
      std::is_same<typename tf2::UnidirectionalTypeMap<A, B>::type, std::nullptr_t>::value &&
      std::is_same<typename tf2::BidirectionalTypeMap<A, B>::type, std::nullptr_t>::value &&
      std::is_same<typename tf2::BidirectionalTypeMap<B, A>::type, std::nullptr_t>::value> {};

// implementation details for sfinae
namespace traits {

template<class A, class B, bool has_type>
struct get_common_bidirectional_type{};

template<class A, class B>
struct get_common_bidirectional_type<A, B, false> {};

template<class A, class B>
struct get_common_bidirectional_type<A, B, true> {
  using type = typename tf2::BidirectionalTypeMap<A, B>::type;
};

template<class A, class B, bool has_type>
struct get_common_unidirectional_type{};

template<class A, class B>
struct get_common_unidirectional_type<A, B, false> {};

template<class A, class B>
struct get_common_unidirectional_type<A, B, true> {
  using type = typename tf2::UnidirectionalTypeMap<A, B>::type;
};
} // namespace traits

// get bidirectional common geometry_msgs type for (A and B)
// Note: check also in reverse order! (get_common_bidirectional_type<B, A>)
template<class A, class B>
struct get_common_bidirectional_type : public traits::get_common_bidirectional_type<A, B,! std::is_same<typename tf2::BidirectionalTypeMap<A, B>::type, std::nullptr_t>::value> {};

// get unidirectional common geometry_msgs type for (A and B)
template<class A, class B>
struct get_common_unidirectional_type : public traits::get_common_unidirectional_type<A, B,! std::is_same<typename tf2::UnidirectionalTypeMap<A, B>::type, std::nullptr_t>::value> {};

// do three-way convert, look up common type for (A and B)
template <class A, class B>
inline void convertViaMessage(const A& a, B& b,typename get_common_bidirectional_type<A, B>::type *c_ptr = nullptr)
{
  // SFINAE will bring the geometry_msgs type as third parameter, extract it
  typename std::remove_pointer<decltype(c_ptr)>::type c;
  fromMsg(toMsg(a, c), b);
}

// do three-way convert, look up common type for (B and A)
template <class A, class B>
inline void convertViaMessage(const A& a, B& b,typename get_common_bidirectional_type<B, A>::type *c_ptr = nullptr)
{
  typename std::remove_pointer<decltype(c_ptr)>::type c;
  fromMsg(toMsg(a, c), b);
}

// do three-way convert, look up common type for (A and B), unidirectional
template <class A, class B>
inline void convertViaMessage(const A& a, B& b,typename get_common_unidirectional_type<A, B>::type *c_ptr = nullptr)
{
  // SFINAE will bring the geometry_msgs type as third parameter, extract it
  typename std::remove_pointer<decltype(c_ptr)>::type c;
  fromMsg(toMsg(a, c), b);
}

struct common_type_lookup_failed : std::false_type {};

// Print a nice message if no common type was defined
// use custom return type to make the selection of this overload testable via decltype
template<class A, class B>
common_type_lookup_failed convertViaMessage(const A&, B&, typename std::enable_if<has_no_common_msgs<A, B>::value, void*>::type = nullptr)
{
  static_assert(! has_no_common_msgs<A, B>::value,
      "Please add a tf2::BidirectionalTypeMap or tf2::UnidirectionalTypeMap specialisation for types A and B.");
  return common_type_lookup_failed();
}


template <bool IS_MESSAGE_A, bool IS_MESSAGE_B>
class Converter {
public:
  template<typename A, typename B>
  static void convert(const A& a, B& b);
};

// The case where both A and B are messages should not happen: if you have two
// messages that are interchangeable, well, that's against the ROS purpose:
// only use one type. Worst comes to worst, specialize the original convert
// function for your types.
// if B == A, the templated version of convert with only one argument will be
// used.
//
//template <>
//template <typename A, typename B>
//inline void Converter<true, true>::convert(const A& a, B& b);

template <>
template <typename A, typename B>
inline void Converter<true, false>::convert(const A& a, B& b)
{
  fromMsg(a, b);
}

template <>
template <typename A, typename B>
inline void Converter<false, true>::convert(const A& a, B& b)
{
  toMsg(a, b);
}

template <>
template <typename A, typename B>
inline void Converter<false, false>::convert(const A& a, B& b)
{
  convertViaMessage(a,b);
}

} // namespace impl
} // namespace tf2

#endif //TF2_IMPL_CONVERT_H
