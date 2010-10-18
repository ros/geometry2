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
#include <tf2/time_cache.h>
#include <sys/time.h>
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include <stdexcept>

#include <geometry_msgs/TransformStamped.h>

std::vector<double> values;
unsigned int step = 0;

void seed_rand()
{
  for (unsigned int i = 0; i < 1000; i++)
  {
    int pseudo_rand = std::floor(i * M_PI);
    values.push_back(( pseudo_rand % 100)/50.0 - 1.0);
    //printf("Seeding with %f\n", values.back());
  }
};


double get_rand() 
{ 
  if (values.size() == 0) throw std::runtime_error("you need to call seed_rand first");
  if (step >= values.size()) 
    step = 0;
  else
    step++;
  return values[step];
}

using namespace tf2;


void setIdentity(geometry_msgs::Transform& trans) 
{
  trans.translation.x = 0;
  trans.translation.y = 0;
  trans.translation.z = 0;
  trans.rotation.x = 0;
  trans.rotation.y = 0;
  trans.rotation.z = 0;
  trans.rotation.w = 1;
}

#if 0

TEST(StaticCache, Repeatability)
{
  unsigned int runs = 100;

  seed_rand();
  
  tf2::StaticCache  cache;

  TransformStorage stor;
  setIdentity(stor.transform);
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    values[i] = 10.0 * ((double) get_rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    stor.header.frame_id = ss.str();
    stor.c_frame_id_ = CompactFrameID(i);
    stor.header.stamp = ros::Time().fromNSec(i);
    
    cache.insertData(stor);

    
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.c_frame_id_.num_, i);
    EXPECT_EQ(stor.header.stamp, ros::Time().fromNSec(i));
    std::stringstream ss2;
    ss2 << values[i];
    EXPECT_EQ(stor.header.frame_id, ss2.str());
    
  }
}

TEST(StaticCache, DuplicateEntries)
{

  tf2::StaticCache cache;

  TransformStorage stor;
  setIdentity(stor.transform); 
  stor.header.frame_id = "a";
  stor.c_frame_id_ = CompactFrameID(3);
  stor.header.stamp = ros::Time().fromNSec(1);

  cache.insertData(stor);

  cache.insertData(stor);


  cache.getData(ros::Time().fromNSec(1), stor);
  
  //printf(" stor is %f\n", stor.transform.translation.x);
  EXPECT_TRUE(!std::isnan(stor.transform.translation.x));
  EXPECT_TRUE(!std::isnan(stor.transform.translation.y));
  EXPECT_TRUE(!std::isnan(stor.transform.translation.z));
  EXPECT_TRUE(!std::isnan(stor.transform.rotation.x));
  EXPECT_TRUE(!std::isnan(stor.transform.rotation.y));
  EXPECT_TRUE(!std::isnan(stor.transform.rotation.z));
  EXPECT_TRUE(!std::isnan(stor.transform.rotation.w));
}

#endif

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
