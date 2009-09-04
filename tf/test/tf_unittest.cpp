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


using namespace tf;

TEST(tf, setTransformNoInsertOnSelfTransform)
{
  tf::Transformer mTR(true);
  Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10.0), "same_frame", "same_frame");
  EXPECT_FALSE(mTR.setTransform(tranStamped));
}

TEST(tf, setTransformNoInsertWithNan)
{
  tf::Transformer mTR(true);
  Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10.0), "same_frame", "other_frame");
  EXPECT_TRUE(mTR.setTransform(tranStamped));

  tranStamped.setOrigin(tf::Point(1.0,1.0,0.0/0.0));
  EXPECT_TRUE(std::isnan(tranStamped.getOrigin().z()));
  EXPECT_FALSE(mTR.setTransform(tranStamped));

}

TEST(tf, setTransformNoInsertWithNoFrameID)
{
  tf::Transformer mTR(true);
  Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10.0), "", "parent_frame");
  EXPECT_FALSE(mTR.setTransform(tranStamped));
}

TEST(tf, setTransformNoInsertWithNoParentID)
{
  tf::Transformer mTR(true);
  Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10.0), "my_frame", "");
  EXPECT_FALSE(mTR.setTransform(tranStamped));
}

TEST(tf, TransformTransformsCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10.0 + i), "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10.0 + i), "child");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
  Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(runs), "child");
  Stamped<Pose> outpose;
  outpose.setIdentity(); //to make sure things are getting mutated
  mTR.transformPose("child",inpose, outpose);
  EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
  
  
}

/** Make sure that the edge cases of transform the top of the tree to the top of the tree and 
 * the leaf of the tree can transform to the leaf of the tree without a lookup exception and accurately */
TEST(tf, TransformTransformToOwnFrame)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs), yawvalues(runs),  pitchvalues(runs),  rollvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yawvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    pitchvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    rollvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(yawvalues[i],pitchvalues[i],rollvalues[i]), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child", "parent");
    mTR.setTransform(tranStamped);

  }

  //std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "child");
    Stamped<btTransform> inpose2 (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "parent");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
    EXPECT_NEAR(outpose.getRotation().w(), 1, epsilon); //Identity is 0,0,0,1


    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("parent",inpose2, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
    EXPECT_NEAR(outpose.getRotation().w(), 1, epsilon); //Identity is 0,0,0,1
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
  Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(runs), "child");
  Stamped<Pose> outpose;
  outpose.setIdentity(); //to make sure things are getting mutated
  mTR.transformPose("child",inpose, outpose);
  EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
  
  
}

TEST(tf, TransformPointCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    double x =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double y =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double z =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    Stamped<Point> invec (btVector3(x,y,z), ros::Time().fromNSec(10 + i), "child");

    try{
      Stamped<Point> outvec(btVector3(0,0,0), ros::Time().fromNSec(10 + i), "child");
    //    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPoint("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.x(), xvalues[i]+x, epsilon);
    EXPECT_NEAR(outvec.y(), yvalues[i]+y, epsilon);
    EXPECT_NEAR(outvec.z(), zvalues[i]+z, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformVectorCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    double x =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double y =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    double z =10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    Stamped<Point> invec (btVector3(x,y,z), ros::Time().fromNSec(10 + i), "child");

    try{
      Stamped<Vector3> outvec(btVector3(0,0,0), ros::Time().fromNSec(10 + i), "child");
    //    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformVector("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.x(), x, epsilon);
    EXPECT_NEAR(outvec.y(), y, epsilon);
    EXPECT_NEAR(outvec.z(), z, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformQuaternionCartesian)
{
  uint64_t runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;


    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child", "my_parent");
    mTR.setTransform(tranStamped);

  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btQuaternion> invec (btQuaternion(xvalues[i],yvalues[i],zvalues[i]), ros::Time().fromNSec(10 + i), "child");
    //    printf("%f, %f, %f\n", xvalues[i],yvalues[i], zvalues[i]);

    try{
      Stamped<btQuaternion> outvec(btQuaternion(xvalues[i],yvalues[i],zvalues[i]), ros::Time().fromNSec(10 + i), "child");

    mTR.transformQuaternion("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.angle(invec) , 0, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(data, Vector3Conversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    btVector3 btv = btVector3(xvalues[i], yvalues[i], zvalues[i]);
    btVector3 btv_out = btVector3(0,0,0);
    geometry_msgs::Vector3 msgv;
    vector3TFToMsg(btv, msgv);
    vector3MsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
  } 
  
}

TEST(data, Vector3StampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<btVector3> btv = Stamped<btVector3>(btVector3(xvalues[i], yvalues[i], zvalues[i]), ros::Time().fromNSec(1), "no frame");
    Stamped<btVector3> btv_out;
    geometry_msgs::Vector3Stamped msgv;
    vector3StampedTFToMsg(btv, msgv);
    vector3StampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, QuaternionConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    btQuaternion btv = btQuaternion(xvalues[i], yvalues[i], zvalues[i]);
    btQuaternion btv_out = btQuaternion(0,0,0);
    geometry_msgs::Quaternion msgv;
    quaternionTFToMsg(btv, msgv);
    quaternionMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_NEAR(btv.w(), btv_out.w(), epsilon);
  } 
  
}

TEST(data, QuaternionStampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<btQuaternion> btv = Stamped<btQuaternion>(btQuaternion(xvalues[i], yvalues[i], zvalues[i]), ros::Time().fromNSec(1), "no frame");
    Stamped<btQuaternion> btv_out;
    geometry_msgs::QuaternionStamped msgv;
    quaternionStampedTFToMsg(btv, msgv);
    quaternionStampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_NEAR(btv.w(), btv_out.w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, TransformConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    btTransform btv = btTransform(btQuaternion(xvalues2[i], yvalues2[i], zvalues2[i]), btVector3(xvalues[i], yvalues[i], zvalues[i]));
    btTransform btv_out;
    geometry_msgs::Transform msgv;
    transformTFToMsg(btv, msgv);
    transformMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.getOrigin().x(), btv_out.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.getOrigin().y(), btv_out.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.getOrigin().z(), btv_out.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().x(), btv_out.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.getRotation().y(), btv_out.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.getRotation().z(), btv_out.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().w(), btv_out.getRotation().w(), epsilon);
  } 
  
}

TEST(data, TransformStampedConversions)
{
  
  uint64_t runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    Stamped<btTransform> btv = Stamped<btTransform>(btTransform(btQuaternion(xvalues2[i], yvalues2[i], zvalues2[i]), btVector3(xvalues[i], yvalues[i], zvalues[i])), ros::Time().fromNSec(1), "no frame");
    Stamped<btTransform> btv_out;
    geometry_msgs::TransformStamped msgv;
    transformStampedTFToMsg(btv, msgv);
    transformStampedMsgToTF(msgv, btv_out);
    EXPECT_NEAR(btv.getOrigin().x(), btv_out.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.getOrigin().y(), btv_out.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.getOrigin().z(), btv_out.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().x(), btv_out.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.getRotation().y(), btv_out.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.getRotation().z(), btv_out.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().w(), btv_out.getRotation().w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(tf, ListOneInverse)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child",  "my_parent");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "child");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, ListTwoInverse)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child",  "my_parent");
    mTR.setTransform(tranStamped);
    Stamped<btTransform> tranStamped2(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "grandchild",  "child");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "grandchild");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 2*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 2*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 2*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}


TEST(tf, ListOneForward)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child",  "my_parent");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "my_parent");

    try{
    Stamped<btTransform> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), -xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), -yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), -zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, ListTwoForward)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "child",  "my_parent");
    mTR.setTransform(tranStamped);
    Stamped<btTransform> tranStamped2(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "grandchild",  "child");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "my_parent");

    try{
    Stamped<btTransform> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("grandchild",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), -2*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), -2*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), -2*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformThrougRoot)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(1000 + i*100), "childA",  "my_parent");
    mTR.setTransform(tranStamped);
    Stamped<btTransform> tranStamped2(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(1000 + i*100), "childB",  "my_parent");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000 + i*100), "childA");

    try{
    Stamped<btTransform> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("childB",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformThroughNO_PARENT)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "childA",  "my_parentA");
    mTR.setTransform(tranStamped);
    Stamped<btTransform> tranStamped2(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i), "childB",  "my_parentB");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "childA");
    bool exception_thrown = false;

    try{
    Stamped<btTransform> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("childB",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }
  
}


TEST(tf, getParent)
{
  
  std::vector<std::string> children;
  std::vector<std::string> parents;

  children.push_back("a");
  parents.push_back("c");

  children.push_back("b");
  parents.push_back("c");

  children.push_back("c");
  parents.push_back("e");

  children.push_back("d");
  parents.push_back("e");

  children.push_back("e");
  parents.push_back("f");

  children.push_back("f");
  parents.push_back("j");

  tf::Transformer mTR(true);

  for (uint64_t i = 0; i <  children.size(); i++)
    {
      Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10) , children[i],  parents[i]);
      mTR.setTransform(tranStamped);
    }

  //std::cout << mTR.allFramesAsString() << std::endl;

  std::string output;
  for  (uint64_t i = 0; i <  children.size(); i++)
    {
      EXPECT_TRUE(mTR.getParent(children[i], ros::Time().fromNSec(10), output));
      EXPECT_STREQ(tf::remap("",parents[i]).c_str(), output.c_str());
    }
  
  EXPECT_FALSE(mTR.getParent("j", ros::Time().fromNSec(10), output));

  EXPECT_FALSE(mTR.getParent("no_value", ros::Time().fromNSec(10), output));
  
}


TEST(tf, NO_PARENT_SET)
{
  double epsilon = 1e-6;
  
  std::vector<std::string> children;
  std::vector<std::string> parents;



  children.push_back("b");
  parents.push_back("a");
  children.push_back("a");
  parents.push_back("NO_PARENT");

  tf::Transformer mTR(true);

  for (uint64_t i = 0; i <  children.size(); i++)
    {
      Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10) , children[i],  parents[i]);
      mTR.setTransform(tranStamped);
    }

  //std::cout << mTR.allFramesAsString() << std::endl;


  Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10), "a");
  Stamped<btTransform> outpose;
  outpose.setIdentity(); //to make sure things are getting mutated
  mTR.transformPose("a",inpose, outpose);
  EXPECT_NEAR(outpose.getOrigin().x(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().y(), 0, epsilon);
  EXPECT_NEAR(outpose.getOrigin().z(), 0, epsilon);
  
}

TEST(tf, waitForTransform)
{
  tf::Transformer mTR(true);

  //A seperate thread is required to use the blocking call for normal usage
  mTR.setUsingDedicatedThread(true);  
  // make sure timeout is resonably lengthed

  ros::Duration timeout = ros::Duration().fromSec(1.0);
  ros::Duration poll_freq = ros::Duration().fromSec(0.1);
  double eps = 0.2;

  // Default polling freq
  ros::Time start_time = ros::Time::now();
  EXPECT_FALSE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000), timeout));
  ros::Time stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)-timeout).toSec()) < eps);

  // 10Hz polling
  start_time = ros::Time::now();
  EXPECT_FALSE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000), timeout, poll_freq));
  stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)-timeout).toSec()) < eps);
  

  //Now it should be able to transform
  mTR.setTransform( Stamped<btTransform>(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10000000) , "me",  "parent"));
  
  start_time = ros::Time::now();
  EXPECT_TRUE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000),timeout));
  stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)).toSec()) < eps);


  start_time = ros::Time::now();
  EXPECT_TRUE(mTR.waitForTransform("parent", "me", ros::Time().fromNSec(10000000),timeout, poll_freq));
  stop_time = ros::Time::now();
  EXPECT_TRUE(fabs(((stop_time-start_time)).toSec()) < eps);
}


TEST(tf, Exceptions)
{

 tf::Transformer mTR(true);

 
 Stamped<btTransform> outpose;

 //connectivity when no data
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(10000000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose>(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10000000) , "me"), outpose);
   EXPECT_FALSE("ConnectivityException Not Thrown");   
 }
 catch ( tf::LookupException &ex)
 {
   EXPECT_TRUE("Lookupgh Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
 
 mTR.setTransform( Stamped<btTransform>(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(100000) , "me",  "parent"));

 //Extrapolation not valid with one value
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(200000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose>(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(200000) , "me"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
 

 mTR.setTransform( Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(300000) , "me",  "parent"));

 //NO Extration when Interpolating
 //inverse list
 EXPECT_TRUE(mTR.canTransform("parent", "me", ros::Time().fromNSec(200000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose>(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(200000) , "me"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_FALSE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }



 //forward list
 EXPECT_TRUE(mTR.canTransform("me", "parent", ros::Time().fromNSec(200000)));
 try 
 {
   mTR.transformPose("me",Stamped<Pose>(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(200000) , "parent"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_FALSE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
  

 //Extrapolating backwards
 //inverse list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(1000)));
 try 
 {
   mTR.transformPose("parent",Stamped<Pose> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000) , "me"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
 //forwards list
 EXPECT_FALSE(mTR.canTransform("me", "parent", ros::Time().fromNSec(1000)));
 try 
 {
   mTR.transformPose("me",Stamped<Pose> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000) , "parent"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
  


 // Test extrapolation inverse and forward linkages FORWARD

 //inverse list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(350000)));
 try 
 {
   mTR.transformPose("parent", Stamped<Pose> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(350000) , "me"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }

 //forward list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(350000)));
 try 
 {
   mTR.transformPose("me", Stamped<Pose> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(350000) , "parent"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
  



}



TEST(tf, NoExtrapolationExceptionFromParent)
{
  tf::Transformer mTR(true, ros::Duration().fromNSec(1000000));
  


mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000), "a",  "parent"));
mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10000), "a",  "parent"));


mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000), "b",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10000), "b",  "parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000), "parent",  "parent's parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000), "parent's parent",  "parent's parent's parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10000), "parent",  "parent's parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(10000), "parent's parent",  "parent's parent's parent"));

  Stamped<Point> output;

  try
  {
    mTR.transformPoint( "b", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(2000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    EXPECT_FALSE("Shouldn't have gotten this exception");
  }



};



TEST(tf, ExtrapolationFromOneValue)
{
  tf::Transformer mTR(true, ros::Duration().fromNSec(1000000));
  


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000), "a",  "parent"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000), "parent",  "parent's parent"));


  Stamped<Point> output;

  bool excepted = false;
  //Past time
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  excepted = false;
  //Future one element
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(100000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  //Past multi link
  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(1), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  //Future case multi link
  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_TRUE(excepted);

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(20000), "a",  "parent"));

  excepted = false;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }
  
  EXPECT_FALSE(excepted);

};



TEST(tf, getLatestCommonTime)
{
  tf::Transformer mTR(true);
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(1000), "a",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(2000), "parent",  "parent's parent"));
  
  //simple case
  ros::Time t;
  mTR.getLatestCommonTime("a", "parent's parent", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(1000));

  //no connection
  EXPECT_EQ(tf::LOOKUP_ERROR, mTR.getLatestCommonTime("a", "not valid", t, NULL));
  EXPECT_EQ(t, ros::Time());

  //testing with update
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(3000), "a",  "parent"));
  mTR.getLatestCommonTime("a", "parent's parent",t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(2000));

  //longer chain
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(4000), "b",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(3000), "c",  "b"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(9000), "d",  "c"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time().fromNSec(5000), "e",  "f"));

  //shared parent
  mTR.getLatestCommonTime("a", "b",t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));

  //two degrees
  mTR.getLatestCommonTime("a", "c", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));
  //reversed
  mTR.getLatestCommonTime("c", "a", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));

  //three degrees
  mTR.getLatestCommonTime("a", "d", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));
  //reversed
  mTR.getLatestCommonTime("d", "a", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));

  //disconnected tree
  mTR.getLatestCommonTime("e", "f", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(5000));
  //reversed order
  mTR.getLatestCommonTime("f", "e", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(5000));


  mTR.setExtrapolationLimit(ros::Duration().fromNSec(20000));

  //check timestamps resulting
  tf::Stamped<tf::Point> output, output2;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time(), "b"), output);
    mTR.transformPoint( "a", ros::Time(),Stamped<Point>(Point(1,1,1), ros::Time(), "b"), "c",  output2);
  }
  catch (tf::TransformException &ex)
  {
    printf("%s\n", ex.what());
    EXPECT_FALSE("Shouldn't get this Exception");
  }

  EXPECT_EQ(output.stamp_, ros::Time().fromNSec(4000));
  EXPECT_EQ(output2.stamp_, ros::Time().fromNSec(3000));


  //zero length lookup zero time
  ros::Time now1 = ros::Time::now();
  ros::Time time_output;
  mTR.getLatestCommonTime("a", "a", time_output, NULL);
  EXPECT_LE(now1.toSec(), time_output.toSec());
  EXPECT_LE(time_output.toSec(), ros::Time::now().toSec());


}

TEST(tf, RepeatedTimes)
{
  Transformer mTR;
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromNSec(4000), "b",  "parent"));
  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,1,0), btVector3(0,0,0)), ros::Time().fromNSec(4000), "b",  "parent"));

  tf::Stamped<tf::Transform>  output;
  try{
    mTR.lookupTransform("parent", "b" , ros::Time().fromNSec(4000), output);
    EXPECT_TRUE(!std::isnan(output.getOrigin().x()));
    EXPECT_TRUE(!std::isnan(output.getOrigin().y()));
    EXPECT_TRUE(!std::isnan(output.getOrigin().z()));
    EXPECT_TRUE(!std::isnan(output.getRotation().x()));
    EXPECT_TRUE(!std::isnan(output.getRotation().y()));
    EXPECT_TRUE(!std::isnan(output.getRotation().z()));
    EXPECT_TRUE(!std::isnan(output.getRotation().w()));
  }
  catch (...)
  {
    EXPECT_FALSE("Excetion improperly thrown");
  }
  

}

TEST(tf, frameExists)
{
  Transformer mTR;

  // test with fully qualified name
  EXPECT_FALSE(mTR.frameExists("/b"));;
  EXPECT_FALSE(mTR.frameExists("/parent"));
  EXPECT_FALSE(mTR.frameExists("/other"));
  EXPECT_FALSE(mTR.frameExists("/frame"));

  //test with remapping
  EXPECT_FALSE(mTR.frameExists("b"));;
  EXPECT_FALSE(mTR.frameExists("parent"));
  EXPECT_FALSE(mTR.frameExists("other"));
  EXPECT_FALSE(mTR.frameExists("frame"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromNSec(4000), "/b",  "/parent"));

  // test with fully qualified name
  EXPECT_TRUE(mTR.frameExists("/b"));
  EXPECT_TRUE(mTR.frameExists("/parent"));
  EXPECT_FALSE(mTR.frameExists("/other"));
  EXPECT_FALSE(mTR.frameExists("/frame"));

  //Test with remapping
  EXPECT_TRUE(mTR.frameExists("b"));
  EXPECT_TRUE(mTR.frameExists("parent"));
  EXPECT_FALSE(mTR.frameExists("other"));
  EXPECT_FALSE(mTR.frameExists("frame"));

  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,1,0), btVector3(0,0,0)), ros::Time().fromNSec(4000), "/other",  "/frame"));

  // test with fully qualified name
  EXPECT_TRUE(mTR.frameExists("/b"));
  EXPECT_TRUE(mTR.frameExists("/parent"));
  EXPECT_TRUE(mTR.frameExists("/other"));
  EXPECT_TRUE(mTR.frameExists("/frame"));
  
  //Test with remapping
  EXPECT_TRUE(mTR.frameExists("b"));
  EXPECT_TRUE(mTR.frameExists("parent"));
  EXPECT_TRUE(mTR.frameExists("other"));
  EXPECT_TRUE(mTR.frameExists("frame"));

}

TEST(tf, remap)
{
  //no prefix
  EXPECT_STREQ("/id", tf::remap("","id").c_str());
  //prefix w/o /
  EXPECT_STREQ("/asdf/id", tf::remap("asdf","id").c_str());
  //prefix w /
  EXPECT_STREQ("/asdf/id", tf::remap("/asdf","id").c_str());
  // frame_id w / -> no prefix
  EXPECT_STREQ("/id", tf::remap("asdf","/id").c_str());
  // frame_id w / -> no prefix
  EXPECT_STREQ("/id", tf::remap("/asdf","/id").c_str());

}

TEST(tf, canTransform)
{
  Transformer mTR;

  //confirm zero length list disconnected will return true
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", ros::Time()));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", ros::Time::now()));

  //Create a two link tree between times 10 and 20
  for (int i = 10; i < 20; i++)
  {
    mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i), "child",  "parent"));
    mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i), "other_child",  "parent"));
  }

  // four different timestamps related to tf state
  ros::Time zero_time = ros::Time().fromSec(0);
  ros::Time old_time = ros::Time().fromSec(5);
  ros::Time valid_time = ros::Time().fromSec(15);
  ros::Time future_time = ros::Time().fromSec(25);


  //confirm zero length list disconnected will return true
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", zero_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", old_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", valid_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", future_time));

  // Basic API Tests

  //Valid data should pass
  EXPECT_TRUE(mTR.canTransform("child", "parent", valid_time));
  EXPECT_TRUE(mTR.canTransform("child", "other_child", valid_time));

  //zero data should pass
  EXPECT_TRUE(mTR.canTransform("child", "parent", zero_time));
  EXPECT_TRUE(mTR.canTransform("child", "other_child", zero_time));

  //Old data should fail
  EXPECT_FALSE(mTR.canTransform("child", "parent", old_time));
  EXPECT_FALSE(mTR.canTransform("child", "other_child", old_time));

  //Future data should fail
  EXPECT_FALSE(mTR.canTransform("child", "parent", future_time));
  EXPECT_FALSE(mTR.canTransform("child", "other_child", future_time));

  //Same Frame should pass for all times
  EXPECT_TRUE(mTR.canTransform("child", "child", zero_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", old_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", valid_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", future_time));

  // Advanced API Tests

  // Source = Fixed
  //zero data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "other_child", valid_time, "child"));
  //Old data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", old_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", old_time, "other_child", valid_time, "child"));
  //valid data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "other_child", valid_time, "child"));
  //future data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", future_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", future_time, "other_child", valid_time, "child"));

  //transforming through fixed into the past
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "parent", old_time, "child"));
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "other_child", old_time, "child"));
  //transforming through fixed into the future
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "parent", future_time, "child"));
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "other_child", future_time, "child"));

  // Target = Fixed
  //zero data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "parent", valid_time, "parent"));
  //Old data in fixed frame should pass
  EXPECT_FALSE(mTR.canTransform("child", old_time, "parent", valid_time, "parent"));
  //valid data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "parent"));
  //future data in fixed frame should pass
  EXPECT_FALSE(mTR.canTransform("child", future_time, "parent", valid_time, "parent"));

  //transforming through fixed into the zero
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", zero_time, "parent"));
  //transforming through fixed into the past
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", old_time, "parent"));
  //transforming through fixed into the valid
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "parent"));
  //transforming through fixed into the future
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", future_time, "parent"));

}

TEST(tf, lookupTransform)
{
  Transformer mTR;
  //Create a two link tree between times 10 and 20
  for (int i = 10; i < 20; i++)
  {
    mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i), "child",  "parent"));
    mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i), "other_child",  "parent"));
  }

  // four different timestamps related to tf state
  ros::Time zero_time = ros::Time().fromSec(0);
  ros::Time old_time = ros::Time().fromSec(5);
  ros::Time valid_time = ros::Time().fromSec(15);
  ros::Time future_time = ros::Time().fromSec(25);

  //output
  tf::Stamped<tf::Transform> output;

  // Basic API Tests

  try
  {
    //confirm zero length list disconnected will return true
    mTR.lookupTransform("some_frame","some_frame", zero_time, output);
    mTR.lookupTransform("some_frame","some_frame", old_time, output);
    mTR.lookupTransform("some_frame","some_frame", valid_time, output);
    mTR.lookupTransform("some_frame","some_frame", future_time, output);
    mTR.lookupTransform("child","child", future_time, output);
    mTR.lookupTransform("other_child","other_child", future_time, output);

    //Valid data should pass
    mTR.lookupTransform("child", "parent", valid_time, output);
    mTR.lookupTransform("child", "other_child", valid_time, output);
    
    //zero data should pass
    mTR.lookupTransform("child", "parent", zero_time, output);
    mTR.lookupTransform("child", "other_child", zero_time, output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception thrown");
  }
  try
  {
    //Old data should fail
    mTR.lookupTransform("child", "parent", old_time, output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
  try {
    //Future data should fail
    mTR.lookupTransform("child", "parent", future_time, output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
    
  try {
    //Same Frame should pass for all times
    mTR.lookupTransform("child", "child", zero_time, output);
    mTR.lookupTransform("child", "child", old_time, output);
    mTR.lookupTransform("child", "child", valid_time, output);
    mTR.lookupTransform("child", "child", future_time, output);
    
    // Advanced API Tests
    
    // Source = Fixed
    //zero data in fixed frame should pass
    mTR.lookupTransform("child", zero_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", zero_time, "other_child", valid_time, "child", output);
    //Old data in fixed frame should pass
    mTR.lookupTransform("child", old_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", old_time, "other_child", valid_time, "child", output);
    //valid data in fixed frame should pass
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", valid_time, "other_child", valid_time, "child", output);
    //future data in fixed frame should pass
    mTR.lookupTransform("child", future_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", future_time, "other_child", valid_time, "child", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception incorrectly thrown");
  }

  try {
    //transforming through fixed into the past
    mTR.lookupTransform("child", valid_time, "parent", old_time, "child", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    //transforming through fixed into the future
    mTR.lookupTransform("child", valid_time, "parent", future_time, "child", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    // Target = Fixed
    //zero data in fixed frame should pass
    mTR.lookupTransform("child", zero_time, "parent", valid_time, "parent", output);
    //valid data in fixed frame should pass
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "parent", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception incorrectly thrown");
  }

  try {
  //Old data in fixed frame should pass
  mTR.lookupTransform("child", old_time, "parent", valid_time, "parent", output);
      EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
  try {
    //future data in fixed frame should pass
    mTR.lookupTransform("child", future_time, "parent", valid_time, "parent", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    //transforming through fixed into the zero
    mTR.lookupTransform("child", valid_time, "parent", zero_time, "parent", output);
    //transforming through fixed into the past
    mTR.lookupTransform("child", valid_time, "parent", old_time, "parent", output);
    //transforming through fixed into the valid
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "parent", output);
    //transforming through fixed into the future
    mTR.lookupTransform("child", valid_time, "parent", future_time, "parent", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception improperly thrown");
  }
  

  //make sure zero goes to now for zero length
  try
  {
    ros::Time now1 = ros::Time::now();

    mTR.lookupTransform("a", "a", ros::Time(),output);
    EXPECT_LE(now1.toSec(), output.stamp_.toSec());
    EXPECT_LE(output.stamp_.toSec(), ros::Time::now().toSec());
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception improperly thrown");
  }
  
}


TEST(tf, getFrameStrings)
{
  Transformer mTR;


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromNSec(4000), "/b",  "/parent"));
  std::vector <std::string> frames_string;
  mTR.getFrameStrings(frames_string);
  ASSERT_EQ(frames_string.size(), 2);
  EXPECT_STREQ(frames_string[0].c_str(), std::string("/b").c_str());
  EXPECT_STREQ(frames_string[1].c_str(), std::string("/parent").c_str());


  mTR.setTransform(  Stamped<btTransform> (btTransform(btQuaternion(1,1,0), btVector3(0,0,0)), ros::Time().fromNSec(4000), "/other",  "/frame"));
  
  mTR.getFrameStrings(frames_string);
  ASSERT_EQ(frames_string.size(), 4);
  EXPECT_STREQ(frames_string[0].c_str(), std::string("/b").c_str());
  EXPECT_STREQ(frames_string[1].c_str(), std::string("/parent").c_str());
  EXPECT_STREQ(frames_string[2].c_str(), std::string("/other").c_str());
  EXPECT_STREQ(frames_string[3].c_str(), std::string("/frame").c_str());

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
