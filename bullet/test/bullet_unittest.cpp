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
#include <sys/time.h>
#include "LinearMath/btTransform.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btVector3.h"

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};


TEST(Bullet, EulerConventions)
{
  double epsilon = 1e-6;
  double yaw, pitch, roll;

  /** Bullet Convention
   * x  left
   * y up
   * z forward 
   *
   * yaw about y
   * then
   * pitch around x
   * then
   * roll around z 
   */


  //Yaw by 90 
  yaw = M_PI/2;
  pitch = 0;
  roll = 0;
  
  btQuaternion q(0,0,0);
  btVector3 v(1,0,0);
  btTransform tr(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  btVector3 v2 = tr * v;

  printf("%f, %f, %f: yaw by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), yaw, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 1,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 
                 
  //Pitch by 90
  yaw = 0;
  pitch = M_PI/2;
  roll = 0;
  
  q = btQuaternion(0,0,0);
  v = btVector3(1,0,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: pitch by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), pitch, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), -1,epsilon); 
                 
  //Roll by 90
  yaw = 0;
  pitch = 0;
  roll = M_PI/2;
  
  q = btQuaternion(0,0,0);
  v = btVector3(0,1,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: roll by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), roll, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), 1,epsilon); 
                 
  //Yaw and Roll by 90
  yaw = M_PI/2;
  pitch = 0;
  roll = M_PI/2;
  
  q = btQuaternion(0,0,0);
  v = btVector3(1,0,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: yaw and roll by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), roll, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 1,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 

  q = btQuaternion(0,0,0);
  v = btVector3(0,1,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: yaw and roll by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), roll, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), 1,epsilon); 

  q = btQuaternion(0,0,0);
  v = btVector3(0,0,1);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: yaw and roll by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), roll, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 1,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 

  //Yaw and Pitch
  yaw = M_PI/2;
  pitch = M_PI/2;
  roll = 0;
  
  q = btQuaternion(0,0,0);
  v = btVector3(1,0,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: yaw and pitch by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), pitch, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), -1,epsilon); 

  q = btQuaternion(0,0,0);
  v = btVector3(0,1,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: yaw and pitch by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), pitch, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), -1,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 

  q = btQuaternion(0,0,0);
  v = btVector3(0,0,1);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: yaw and pitch by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), pitch, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 1,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 

  //Pitch and Roll
  yaw = 0;
  pitch = M_PI/2;
  roll = M_PI/2;
  
  q = btQuaternion(0,0,0);
  v = btVector3(1,0,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: pitch and roll by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), pitch, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), -1,epsilon); 

  q = btQuaternion(0,0,0);
  v = btVector3(0,1,0);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: pitch and roll by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), pitch, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 1,epsilon); 
  EXPECT_NEAR(v2.y(), 0,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 

  q = btQuaternion(0,0,0);
  v = btVector3(0,0,1);
  tr = btTransform(btQuaternion(yaw, pitch,roll), btVector3(0,0,0));
  
  v2 = tr * v;

  printf("%f, %f, %f: pitch and roll by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), pitch, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), -1,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 
};


TEST(Bullet, Slerp)
{

  unsigned int runs = 100;
  seed_rand();

  btQuaternion q1, q2;
  q1.setEuler(0,0,0);
  
  for (unsigned int i = 0 ; i < runs ; i++)
  {
    q2.setEuler(1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX,
                1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX,
                1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX);
    
    
    btQuaternion q3 = slerp(q1,q2,0.5);
    
    EXPECT_NEAR(q3.angle(q1), q2.angle(q3), 1e-5);
  }

}

TEST(Bullet, QuaternionMultiplication)
{
  btQuaternion q1 (btVector3(1,0,0), M_PI/2.0);
  btQuaternion q2 (btVector3(0,0,1), M_PI/2.0);

  btQuaternion q3 = q1*q2;
  printf("(%f,%f,%f,%f)*(%f,%f,%f,%f)=(%f,%f,%f,%f)\n",q1.x(), q1.y(), q1.z(), q1.getAngle(), q2.x(), q2.y(), q2.z(), q2.getAngle(), q3.x(), q3.y(), q3.z(), q3.getAngle());

  btMatrix3x3 m3(q3);

  btVector3 xout = m3*btVector3(1,0,0);
  btVector3 yout = m3*btVector3(0,1,0);
  btVector3 zout = m3*btVector3(0,0,1);
  printf("(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", 
         xout.x(), xout.y(), xout.z() ,
         yout.x(), yout.y(), yout.z() ,
         zout.x(), zout.y(), zout.z());

  q3 = q2*q1;
  printf("(%f,%f,%f,%f)*(%f,%f,%f,%f)=(%f,%f,%f,%f)\n",q2.x(), q2.y(), q2.z(), q2.getAngle(), q1.x(), q1.y(), q1.z(), q1.getAngle(), q3.x(), q3.y(), q3.z(), q3.getAngle());

  m3.setRotation(q3);

  xout = m3*btVector3(1,0,0);
  yout = m3*btVector3(0,1,0);
  zout = m3*btVector3(0,0,1);
  printf("(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", 
         xout.x(), xout.y(), xout.z() ,
         yout.x(), yout.y(), yout.z() ,
         zout.x(), zout.y(), zout.z());
}
TEST(Bullet, QuaternionTimesEqual)
{
  btQuaternion q1 (btVector3(1,0,0), M_PI/2.0);
  btQuaternion q2 (btVector3(0,0,1), M_PI/2.0);

  btQuaternion q3 (q1);
  q3*=q2;
  printf("(%f,%f,%f,%f)*(%f,%f,%f,%f)=(%f,%f,%f,%f)\n",q1.x(), q1.y(), q1.z(), q1.getAngle(), q2.x(), q2.y(), q2.z(), q2.getAngle(), q3.x(), q3.y(), q3.z(), q3.getAngle());

  btMatrix3x3 m3(q3);

  btVector3 xout = m3*btVector3(1,0,0);
  btVector3 yout = m3*btVector3(0,1,0);
  btVector3 zout = m3*btVector3(0,0,1);
  printf("(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", 
         xout.x(), xout.y(), xout.z() ,
         yout.x(), yout.y(), yout.z() ,
         zout.x(), zout.y(), zout.z());
  q3 = q1;
  q3*=q1;
  printf("(%f,%f,%f,%f)*(%f,%f,%f,%f)=(%f,%f,%f,%f)\n",q2.x(), q2.y(), q2.z(), q2.getAngle(), q1.x(), q1.y(), q1.z(), q1.getAngle(), q3.x(), q3.y(), q3.z(), q3.getAngle());

  m3.setRotation(q3);

  xout = m3*btVector3(1,0,0);
  yout = m3*btVector3(0,1,0);
  zout = m3*btVector3(0,0,1);
  printf("(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", 
         xout.x(), xout.y(), xout.z() ,
         yout.x(), yout.y(), yout.z() ,
         zout.x(), zout.y(), zout.z());
}

TEST (Bullet, downcast)
{
  btVector3 v (1,2,3);
  btVector3 v2(v); 
  
}

TEST(Bullet, TransformOrder )
{
  btTransform tf(btQuaternion(1,0,0));
  btTransform tf2(btQuaternion(0,0,0),btVector3(100,0,0));

  btTransform tf3 = tf * tf2;

  tf*= tf2;
  EXPECT_TRUE(tf3  == tf);

  tf = btTransform(btQuaternion(1,0,0));
  tf3 = tf;

  EXPECT_TRUE(tf.inverse() * tf2 == tf.inverseTimes(tf2));

  
}


TEST(Bullet, SlerpOppositeSigns)
{
  //  btQuaternion q1 (M_PI/2,0,0);
  btQuaternion q1 (M_PI/4,0,0);
  btQuaternion q2(-q1.x(), -q1.y(), -q1.z()-.0001, -q1.w()+.0001);
  q2.normalize();
  btQuaternion q3 = q2.slerp(q1, .5);

  //  printf("%f %f %f %f,%f %f %f %f\n", q2.x(), q2.y(), q2.z(), q2.w(), q3.x(), q3.y(), q3.z(), q3.w());

  EXPECT_NEAR(q2.x(), q3.x(), 0.01);
  EXPECT_NEAR(q2.y(), q3.y(), 0.01);
  EXPECT_NEAR(q2.z(), q3.z(), 0.01);
  EXPECT_NEAR(q2.w(), q3.w(), 0.01);

}

TEST(Bullet, SetEulerZYX)
{
  btMatrix3x3 mat;
  mat.setEulerZYX(M_PI/2, 0, 0);
  double yaw, pitch, roll;
  mat.getEulerZYX(yaw, pitch, roll);
  EXPECT_NEAR(yaw, M_PI/2, 0.1);
  EXPECT_NEAR(pitch, 0, 0.1);
  EXPECT_NEAR(roll, 0, 0.1);
//  printf("%f %f %f\n", yaw, pitch, roll);
  btQuaternion q;
  mat.getRotation(q);
  EXPECT_NEAR(q.z(), sqrt(2)/2, 0.1);
  EXPECT_NEAR(q.y(), 0, 0.1);
  EXPECT_NEAR(q.x(), 0, 0.1);
  EXPECT_NEAR(q.w(), sqrt(2)/2, 0.1);
  //  printf("%f %f %f %f\n", q.x(), q.y(), q.z(), q.w());
}


TEST(Bullet, calculateDiffAxisAngleQuaternion)
{
  btVector3 vec;
  btScalar ang;
  for (unsigned int i = 0 ; i < 1000 ; i++)
  {
    btQuaternion q1(M_PI*2 *(double) i / 1000, 0, 0);
    btQuaternion q2(M_PI/2*0, 0,0);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q1, q2, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(M_PI*2 *(double) i / 1000, ang, 0.001);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q2, q1, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(M_PI*2 *(double) i / 1000, ang, 0.001);
  }
  for (unsigned int i = 0 ; i < 1000 ; i++)
  {
    btQuaternion q1(0, M_PI*2 *(double) i / 1000,1);
    btQuaternion q2(0, 0, 1);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q1, q2, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(M_PI*2 *(double) i / 1000, ang, 0.001);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q2, q1, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(M_PI*2 *(double) i / 1000, ang, 0.001);
  }
  for (unsigned int i = 0 ; i < 1000 ; i++)
  {
    btQuaternion q1(0, 0, M_PI*2 *(double) i / 1000);
    btQuaternion q2(0, 0,0);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q1, q2, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(M_PI*2 *(double) i / 1000, ang, 0.001);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q2, q1, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(M_PI*2 *(double) i / 1000, ang, 0.001);
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
