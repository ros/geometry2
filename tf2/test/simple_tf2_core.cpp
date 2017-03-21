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

#include <gtest/gtest.h>
#include <tf2/buffer_core.h>
#include <sys/time.h>
#include <ros/ros.h>
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

void generate_rand_vectors(double scale, uint64_t runs, std::vector<double>& xvalues, std::vector<double>& yvalues, std::vector<double>&zvalues)
{
  seed_rand();
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
}



TEST(tf2, setTransformFail)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));
  
}

TEST(tf2, setTransformValid)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  
}

TEST(tf2_lookupTransform, LookupException_Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_THROW(tfc.lookupTransform("a", "b", ros::Time().fromSec(1.0)), tf2::LookupException);
  
}

TEST(tf2_canTransform, Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_FALSE(tfc.canTransform("a", "b", ros::Time().fromSec(1.0)));
  
}

TEST(tf2_lookupTransform, LookupException_One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_THROW(tfc.lookupTransform("foo", "bar", ros::Time().fromSec(1.0)), tf2::LookupException);
  
}

TEST(tf2_canTransform, One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_FALSE(tfc.canTransform("foo", "bar", ros::Time().fromSec(1.0)));
}

void addChild(const std::string& parent, const std::string& child,
              tf2::BufferCore& tfc, bool isStatic = false, double time = 1.0)
{
  geometry_msgs::TransformStamped st;
  st.header.frame_id = parent;
  st.header.stamp = ros::Time(time);
  st.child_frame_id = child;
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1", isStatic));
}

TEST(tf2_NoParent, Child_NoParent)
{
  tf2::BufferCore tfc;
  addChild("root", "child2", tfc);
  addChild("c3", "c4", tfc);
  addChild("NO_PARENT", "c3", tfc);
  addChild("NO_PARENT", "root", tfc);
  EXPECT_FALSE(tfc.canTransform("c3", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("c3", "c4", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("child2", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("c4", "root", ros::Time().fromSec(1.0)));
  addChild("root", "c4", tfc);
  EXPECT_TRUE(tfc.canTransform("c4", "root", ros::Time().fromSec(1.0)));
}

TEST(tf2_erase, Basic_Clear)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_TRUE(tfc.canTransform("child", "foo", ros::Time().fromSec(1.0)));
  tfc.erase("foo");
  EXPECT_FALSE(tfc.canTransform("child", "foo", ros::Time().fromSec(1.0)));
}

// R
// | \
// C1  C2
// |
// C3
// |
// C4
TEST(tf2_erase, Clear_With_Parent)
{
  tf2::BufferCore tfc;
  addChild("root", "child1", tfc);
  addChild("root", "child2", tfc);
  addChild("child1", "child3", tfc);
  addChild("child3", "child4", tfc);
  EXPECT_TRUE(tfc.canTransform("child1", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("child2", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("child3", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("child3", "child2", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("child4", "child3", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc._frameExists("child1"));
  EXPECT_NO_THROW(tfc.lookupTransform("child4", "child3", ros::Time().fromSec(1.0)));
  tfc.erase("child1");
  EXPECT_FALSE(tfc._frameExists("child1"));
  EXPECT_TRUE(tfc.canTransform("child2", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("child4", "child3", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("child1", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("child3", "child1", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("child3", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("child3", "child2", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("child4", "root", ros::Time().fromSec(1.0)));
  addChild("root", "child4", tfc);
  EXPECT_TRUE(tfc.canTransform("child4", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("child4", "child2", ros::Time().fromSec(1.0)));
}

// R
// | \
// S2 B1
// | \  \
// S3 S4 S1
TEST(tf2_erase, Clear_Static_Frame)
{
  tf2::BufferCore tfc;
  addChild("root", "b1", tfc, false);
  addChild("b1", "s1", tfc, true);
  addChild("root", "s2", tfc, true);
  addChild("s2", "s3", tfc, true);
  addChild("s2", "s4", tfc, true);
  EXPECT_TRUE(tfc.canTransform("b1", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s1", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s3", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s4", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s4", "s1", ros::Time().fromSec(1.0)));
  tfc.erase("s2");
  EXPECT_TRUE(tfc.canTransform("b1", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s1", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s3", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s4", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s4", "s1", ros::Time().fromSec(1.0)));
}

// R
// | \
// S2-B1
// | \  \
// S3 S4 S1
TEST(tf2_erase, Clear_Static_Frame_Multiple_Link)
{
  tf2::BufferCore tfc;
  addChild("root", "b1", tfc, false);
  addChild("b1", "s1", tfc, true);
  addChild("root", "s2", tfc, true);
  addChild("s2", "b1", tfc, false, 0.9);
  addChild("s2", "s3", tfc, true);
  addChild("s2", "s4", tfc, true);
  EXPECT_TRUE(tfc.canTransform("b1", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("b1", "s2", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s1", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s3", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s4", "root", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s4", "s1", ros::Time().fromSec(1.0)));
  tfc.erase("s2");
  EXPECT_TRUE(tfc.canTransform("b1", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("b1", "s2", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s1", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s3", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s4", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s4", "s1", ros::Time().fromSec(1.0)));
  tfc.erase("NO_PARENT");
  EXPECT_TRUE(tfc.canTransform("b1", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("b1", "s2", ros::Time().fromSec(1.0)));
  EXPECT_TRUE(tfc.canTransform("s1", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s3", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s4", "root", ros::Time().fromSec(1.0)));
  EXPECT_FALSE(tfc.canTransform("s4", "s1", ros::Time().fromSec(1.0)));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init(); //needed for ros::TIme::now()
  return RUN_ALL_TESTS();
}
