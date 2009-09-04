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
#include <tf/tf.h>
#include <sys/time.h>
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"


void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

using namespace tf;


TEST(TimeCache, Repeatability)
{
  unsigned int runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  TransformStorage stor;
  stor.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    stor.frame_id_ = ss.str();
    stor.parent_frame_id = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }
  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.parent_frame_id, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
    std::stringstream ss;
    ss << values[i];
    EXPECT_EQ(stor.frame_id_, ss.str());
  }
  
}


TEST(TimeCache, RepeatabilityReverseInsertOrder)
{
  unsigned int runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  TransformStorage stor;
  stor.setIdentity();
  
  for ( int i = runs -1; i >= 0 ; i-- )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    stor.frame_id_ = ss.str();
    stor.parent_frame_id = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }
  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.parent_frame_id, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
    std::stringstream ss;
    ss << values[i];
    EXPECT_EQ(stor.frame_id_, ss.str());
  }
  
}

TEST(TimeCache, RepeatabilityRandomInsertOrder)
{

  seed_rand();
  
  tf::TimeCache  cache;
  double my_vals[] = {13,2,5,4,9,7,3,11,15,14,12,1,6,10,0,8};
  std::vector<double> values (my_vals, my_vals + sizeof(my_vals)/sizeof(double)); 
  unsigned int runs = values.size();

  TransformStorage stor;
  stor.setIdentity();
  for ( uint64_t i = 0; i <runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    stor.frame_id_ = ss.str();
    stor.parent_frame_id = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }
  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.parent_frame_id, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
    std::stringstream ss;
    ss << values[i];
    EXPECT_EQ(stor.frame_id_, ss.str());
  }
  
}

TEST(TimeCache, ZeroAtFront)
{
  uint64_t runs = 100;

  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> values(runs);

  TransformStorage stor;
  stor.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    values[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    std::stringstream ss;
    ss << values[i];
    stor.frame_id_ = ss.str();
    stor.parent_frame_id = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }

  stor.frame_id_ = "HEAD";
  stor.parent_frame_id = runs;
  stor.stamp_ = ros::Time().fromNSec(runs);
  cache.insertData(stor);
  


  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.parent_frame_id, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
    std::stringstream ss;
    ss << values[i];
    EXPECT_EQ(stor.frame_id_, ss.str());
  }

  cache.getData(ros::Time(), stor);
  EXPECT_EQ(stor.parent_frame_id, runs);
  EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(runs));
  EXPECT_EQ(stor.frame_id_, std::string("HEAD"));

  stor.frame_id_ = "NEW_HEAD";
  stor.parent_frame_id = runs;
  stor.stamp_ = ros::Time().fromNSec(runs+1);
  cache.insertData(stor);


  //Make sure we get a different value now that a new values is added at the front
  cache.getData(ros::Time(), stor);
  EXPECT_EQ(stor.parent_frame_id, runs);
  EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(runs+1));
  EXPECT_NE(stor.frame_id_, std::string("HEAD"));
  
}

TEST(TimeCache, CartesianInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  uint64_t offset = 200;

  TransformStorage stor;
  stor.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {

    for (uint64_t step = 0; step < 2 ; step++)
    {
      xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      stor.setOrigin(btVector3(xvalues[step], yvalues[step], zvalues[step]));
      stor.frame_id_ = "NO_NEED";
      stor.parent_frame_id = 2;
      stor.stamp_ = ros::Time().fromNSec(step * 100 + offset);
      cache.insertData(stor);
    }
    
    for (int pos = 0; pos < 100 ; pos ++)
    {
      cache.getData(ros::Time().fromNSec(offset + pos), stor);
      double x_out = stor.getOrigin().x();
      double y_out = stor.getOrigin().y();
      double z_out = stor.getOrigin().z();
      EXPECT_NEAR(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.0, x_out, epsilon);
      EXPECT_NEAR(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0, y_out, epsilon);
      EXPECT_NEAR(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos/100.0, z_out, epsilon);
    }
    

    cache.clearList();
  }

  
}

/** \brief Make sure we dont' interpolate across reparented data */
TEST(TimeCache, ReparentingInterpolationProtection)
{
  double epsilon = 1e-6;
  uint64_t offset = 555;

  tf::TimeCache cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  TransformStorage stor;
  stor.setIdentity();

  for (uint64_t step = 0; step < 2 ; step++)
  {
    xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
    stor.setOrigin(btVector3(xvalues[step], yvalues[step], zvalues[step]));
    stor.frame_id_ = "NO_NEED";
    stor.parent_frame_id = step + 4;
    stor.stamp_ = ros::Time().fromNSec(step * 100 + offset);
    cache.insertData(stor);
  }
  
  for (int pos = 0; pos < 100 ; pos ++)
  {
    cache.getData(ros::Time().fromNSec(offset + pos), stor);
    double x_out = stor.getOrigin().x();
    double y_out = stor.getOrigin().y();
    double z_out = stor.getOrigin().z();
    EXPECT_NEAR(xvalues[0], x_out, epsilon);
    EXPECT_NEAR(yvalues[0], y_out, epsilon);
    EXPECT_NEAR(zvalues[0], z_out, epsilon);
  }
  
  for (int pos = 100; pos < 120 ; pos ++)
  {
    cache.getData(ros::Time().fromNSec(offset + pos), stor);
    double x_out = stor.getOrigin().x();
    double y_out = stor.getOrigin().y();
    double z_out = stor.getOrigin().z();
    EXPECT_NEAR(xvalues[1], x_out, epsilon);
    EXPECT_NEAR(yvalues[1], y_out, epsilon);
    EXPECT_NEAR(zvalues[1], z_out, epsilon);
  }


}

TEST(TimeCache, CartesianExtrapolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-5;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  uint64_t offset = 555;

  TransformStorage stor;
  stor.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {

    for (uint64_t step = 0; step < 2 ; step++)
    {
      xvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      yvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      zvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      stor.setOrigin(btVector3(xvalues[step], yvalues[step], zvalues[step]));
      stor.frame_id_ = "NO_NEED";
      stor.parent_frame_id = 2;
      stor.stamp_ = ros::Time().fromNSec(step * 100 + offset);
      cache.insertData(stor);
    }
    
    for (int pos = -200; pos < 300 ; pos ++)
    {
      cache.getData(ros::Time().fromNSec(offset + pos), stor);
      double x_out = stor.getOrigin().x();
      double y_out = stor.getOrigin().y();
      double z_out = stor.getOrigin().z();
      EXPECT_NEAR(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.0, x_out, epsilon);
      EXPECT_NEAR(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0, y_out, epsilon);
      EXPECT_NEAR(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos/100.0, z_out, epsilon);
    }
    
    cache.clearList();
  }

  
}


TEST(Bullet, Slerp)
{

  uint64_t runs = 100;
  seed_rand();

  btQuaternion q1, q2;
  q1.setEuler(0,0,0);
  
  for (uint64_t i = 0 ; i < runs ; i++)
  {
    q2.setEuler(1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX,
                1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX,
                1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX);
    
    
    btQuaternion q3 = slerp(q1,q2,0.5);
    
    EXPECT_NEAR(q3.angle(q1), q2.angle(q3), 1e-5);
  }

}


TEST(TimeCache, AngularInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::TimeCache  cache;
  std::vector<double> yawvalues(2);
  std::vector<double> pitchvalues(2);
  std::vector<double> rollvalues(2);
  uint64_t offset = 200;

  std::vector<btQuaternion> quats(2);

  TransformStorage stor;
  stor.setIdentity();
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {

    for (uint64_t step = 0; step < 2 ; step++)
    {
      yawvalues[step] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX / 100.0;
      pitchvalues[step] = 0;//10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      rollvalues[step] = 0;//10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
      quats[step] = btQuaternion(yawvalues[step], pitchvalues[step], rollvalues[step]);
      stor.setRotation(quats[step]);
      stor.frame_id_ = "NO_NEED";
      stor.parent_frame_id = 3;
      stor.stamp_ = ros::Time().fromNSec(offset + (step * 100)); //step = 0 or 1
      cache.insertData(stor);
    }
    
    for (int pos = -100; pos < 200 ; pos ++)
    {
      cache.getData(ros::Time().fromNSec(offset + pos), stor); //get the transform for the position
      btQuaternion quat = stor.getRotation(); //get the quaternion out of the transform

      //Generate a ground truth quaternion directly calling slerp
      btQuaternion ground_truth = quats[0].slerp(quats[1], pos/100.0);
      
      //Make sure the transformed one and the direct call match
      EXPECT_NEAR(ground_truth.x(), quat.x(), epsilon);
      EXPECT_NEAR(ground_truth.y(), quat.y(), epsilon);
      EXPECT_NEAR(ground_truth.z(), quat.z(), epsilon);
      EXPECT_NEAR(ground_truth.w(), quat.w(), epsilon);
            
    }
    
    cache.clearList();
  }

  
}

TEST(TimeCache, DuplicateEntries)
{

  TimeCache cache;

  TransformStorage stor;
  stor.setIdentity();
  stor.frame_id_ = "a";
  stor.parent_frame_id = 3;
  stor.stamp_ = ros::Time().fromNSec(1);

  cache.insertData(stor);

  cache.insertData(stor);


  cache.getData(ros::Time().fromNSec(1), stor);
  
  printf(" stor is %f\n", stor.getOrigin().x());
  EXPECT_TRUE(!std::isnan(stor.getOrigin().x()));
  EXPECT_TRUE(!std::isnan(stor.getOrigin().y()));
  EXPECT_TRUE(!std::isnan(stor.getOrigin().z()));
  EXPECT_TRUE(!std::isnan(stor.getRotation().x()));
  EXPECT_TRUE(!std::isnan(stor.getRotation().y()));
  EXPECT_TRUE(!std::isnan(stor.getRotation().z()));
  EXPECT_TRUE(!std::isnan(stor.getRotation().w()));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
