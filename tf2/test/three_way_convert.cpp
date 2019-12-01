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

#include <gtest/gtest.h>
#include <tf2/convert.h>
#include <geometry_msgs/Vector3.h>

// some test data

namespace Foo {
  struct Vec3 {
    double x,y,z;
  };

  struct Quaternion {};
}

namespace Bar {
  struct Vec3 {
    double x,y,z;
  };

  struct Quaternion {};
}

namespace Baz {
  struct Vec3 {
    double x,y,z;
  };
}

namespace tf2 {


template<>
inline
void fromMsg(const geometry_msgs::Vector3& msg, Foo::Vec3 &out) {
  out.x = msg.x;
  out.y = msg.y;
  out.z = msg.z;
}

template<>
inline
void fromMsg(const geometry_msgs::Vector3& msg, Bar::Vec3 &out) {
  out.x = msg.x;
  out.y = msg.y;
  out.z = msg.z;
}

template<>
inline
void fromMsg(const geometry_msgs::Vector3& msg, Baz::Vec3 &out) {
  out.x = msg.x;
  out.y = msg.y;
  out.z = msg.z;
}

template<>
inline
geometry_msgs::Vector3& toMsg(const Foo::Vec3 &in, geometry_msgs::Vector3& msg) {
  msg.x = in.x;
  msg.y = in.y;
  msg.z = in.z;
  return msg;
}

template<>
inline
geometry_msgs::Vector3& toMsg(const Bar::Vec3 &in, geometry_msgs::Vector3& msg) {
  msg.x = in.x;
  msg.y = in.y;
  msg.z = in.z;
  return msg;
}

template<>
struct BidirectionalTypeMap<Foo::Vec3, Bar::Vec3> {
  using type = geometry_msgs::Vector3;
};

template<>
struct UnidirectionalTypeMap<Bar::Vec3, Baz::Vec3> {
  using type = geometry_msgs::Vector3;
};

} // namespace tf2

// init vector with integers
template<class V>
V initVec() {
  V v;
  v.x = 2.0; v.y = 3.0; v.z = -4.0;
  return v;
}

template<class ...>
using void_t = void;

// helper to check tf2::impl::get_common_bidirectional_type
template<class A, class B, class = void>
struct get_bi_type_returns_type : std::false_type {};

template<class A, class B>
struct get_bi_type_returns_type<A, B,
  void_t<typename tf2::impl::get_common_bidirectional_type<A,B>::type>>
: std::true_type{};

// helper to check tf2::impl::get_common_unidirectional_type
template<class A, class B, class = void>
struct get_uni_type_returns_type : std::false_type {};

template<class A, class B>
struct get_uni_type_returns_type<A, B,
    void_t<typename tf2::impl::get_common_unidirectional_type<A,B>::type>>
: std::true_type{};

// helper to check whether error message would be shown with static_assert
template<class A, class B>
using would_show_error_msg = std::is_same<
  decltype(tf2::impl::convertViaMessage<A,B>(std::declval<const A&>(), std::declval<B&>())),
  tf2::impl::common_type_lookup_failed>;


TEST(ThreeWayConvert, BidirectionalTypeMap) {
    // check raw structs
  ::testing::StaticAssertTypeEq<typename
    tf2::BidirectionalTypeMap<Foo::Vec3, Bar::Vec3>::type,
    geometry_msgs::Vector3>();
  ::testing::StaticAssertTypeEq<typename
    tf2::BidirectionalTypeMap<Bar::Vec3, Foo::Vec3>::type,
    std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
    tf2::UnidirectionalTypeMap<Foo::Vec3, Bar::Vec3>::type,
    std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
    tf2::UnidirectionalTypeMap<Bar::Vec3, Foo::Vec3>::type,
    std::nullptr_t>();

  // check getters
  EXPECT_TRUE((get_bi_type_returns_type<Foo::Vec3, Bar::Vec3>::value));
  EXPECT_FALSE((get_bi_type_returns_type<Bar::Vec3, Foo::Vec3>::value));
  EXPECT_FALSE((get_uni_type_returns_type<Foo::Vec3, Bar::Vec3>::value));
  EXPECT_FALSE((get_uni_type_returns_type<Bar::Vec3, Foo::Vec3>::value));

  EXPECT_FALSE((tf2::impl::has_no_common_msgs<Foo::Vec3, Bar::Vec3>::value));
  EXPECT_FALSE((tf2::impl::has_no_common_msgs<Bar::Vec3, Foo::Vec3>::value));

  EXPECT_FALSE((would_show_error_msg<Foo::Vec3, Bar::Vec3>::value));

  const auto v1 = initVec<Foo::Vec3>();
  Foo::Vec3 v3;


  Bar::Vec3 v2;

  tf2::convert(v1, v3);
  tf2::convert(v3, v2);
  tf2::convert(v2, v2);
  tf2::convert(v2, v3);

  EXPECT_EQ(v1.x, v3.x);
  EXPECT_EQ(v1.y, v3.y);
  EXPECT_EQ(v1.z, v3.z);
}

TEST(ThreeWayConvert, UnidirectionalTypeMap) {
  // check raw structs
  ::testing::StaticAssertTypeEq<typename
    tf2::BidirectionalTypeMap<Baz::Vec3, Bar::Vec3>::type,
    std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
    tf2::BidirectionalTypeMap<Bar::Vec3, Baz::Vec3>::type,
    std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
    tf2::UnidirectionalTypeMap<Baz::Vec3, Bar::Vec3>::type,
    std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
    tf2::UnidirectionalTypeMap<Bar::Vec3, Baz::Vec3>::type,
    geometry_msgs::Vector3>();

  // check getters
  EXPECT_FALSE((get_bi_type_returns_type<Baz::Vec3, Bar::Vec3>::value));
  EXPECT_FALSE((get_bi_type_returns_type<Bar::Vec3, Baz::Vec3>::value));
  EXPECT_FALSE((get_uni_type_returns_type<Baz::Vec3, Bar::Vec3>::value));
  EXPECT_TRUE((get_uni_type_returns_type<Bar::Vec3, Baz::Vec3>::value));


  EXPECT_FALSE((tf2::impl::has_no_common_msgs<Bar::Vec3, Baz::Vec3>::value));
  EXPECT_TRUE((tf2::impl::has_no_common_msgs<Baz::Vec3, Bar::Vec3>::value));

  EXPECT_FALSE((would_show_error_msg<Bar::Vec3, Baz::Vec3>::value));
  EXPECT_TRUE((would_show_error_msg<Baz::Vec3, Bar::Vec3>::value));

  const auto v1 = initVec<Bar::Vec3>();
  Baz::Vec3 v2;

  tf2::convert(v1, v2);

  EXPECT_EQ(v1.x, v2.x);
  EXPECT_EQ(v1.y, v2.y);
  EXPECT_EQ(v1.z, v2.z);
}

TEST(ThreeWayConvert, UnrelatedTypes) {
  ::testing::StaticAssertTypeEq<typename
      tf2::BidirectionalTypeMap<Foo::Quaternion, Bar::Quaternion>::type,
      std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
      tf2::BidirectionalTypeMap<Bar::Quaternion, Foo::Quaternion>::type,
      std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
      tf2::UnidirectionalTypeMap<Foo::Quaternion, Bar::Quaternion>::type,
      std::nullptr_t>();
  ::testing::StaticAssertTypeEq<typename
      tf2::UnidirectionalTypeMap<Bar::Quaternion, Foo::Quaternion>::type,
      std::nullptr_t>();

  // check getters
  EXPECT_FALSE((get_bi_type_returns_type<Foo::Quaternion, Bar::Quaternion>::value));
  EXPECT_FALSE((get_bi_type_returns_type<Bar::Quaternion, Foo::Quaternion>::value));
  EXPECT_FALSE((get_uni_type_returns_type<Foo::Quaternion, Bar::Quaternion>::value));
  EXPECT_FALSE((get_uni_type_returns_type<Bar::Quaternion, Foo::Quaternion>::value));

  EXPECT_TRUE((tf2::impl::has_no_common_msgs<Foo::Quaternion, Bar::Quaternion>::value));
  EXPECT_TRUE((tf2::impl::has_no_common_msgs<Bar::Quaternion, Foo::Quaternion>::value));

  EXPECT_TRUE((would_show_error_msg<Foo::Quaternion, Bar::Quaternion>::value));
  EXPECT_TRUE((would_show_error_msg<Bar::Quaternion, Foo::Quaternion>::value));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
