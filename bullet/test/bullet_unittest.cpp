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


TEST(Bullet, QuaternionAngleScaling)
{
  double epsilon = 1e-6;

  btQuaternion id(0,0,0,1);
  btQuaternion ninty(0,0,sqrt(2.0)/2.0,sqrt(2.0)/2.0);
  EXPECT_NEAR(id.angle(ninty), SIMD_PI/4, epsilon); //These two quaternions are 90 degrees apart half angle is 45

  btQuaternion id2(0,0,0);
  btQuaternion ninty2(SIMD_PI/2,0,0);
  EXPECT_NEAR(id2.angle(ninty2), SIMD_PI/4, epsilon); //These two quaternions are 90 degrees apart half angle is 45
}
TEST(Bullet, QuaternionAngleShortestPath)
{
  btQuaternion q1 (SIMD_PI/4,0,0);
  btQuaternion q2(-q1.x(), -q1.y(), -q1.z()-.0001, -q1.w()+.0001);
  q2.normalize();
  //  printf("%f %f %f %f,%f %f %f %f\n", q2.x(), q2.y(), q2.z(), q2.w(), q3.x(), q3.y(), q3.z(), q3.w());

  EXPECT_NEAR(q2.angleShortestPath(q1), 0, 0.01); //These two quaternions are basically the same but expressed on the other side of quaternion space
}

TEST(Bullet, QuaternionAngleIdentityShortestPath)
{
  btQuaternion q1 (SIMD_PI/4,0,0);
  btQuaternion q2(-q1.x(), -q1.y(), -q1.z(), -q1.w());
  q2.normalize();
  //  printf("%f %f %f %f,%f %f %f %f\n", q2.x(), q2.y(), q2.z(), q2.w(), q3.x(), q3.y(), q3.z(), q3.w());

  EXPECT_NEAR(q2.angleShortestPath(q1), 0, 0.01); //These two quaternions are basically the same but expressed on the other side of quaternion space
  EXPECT_NEAR((q2*(q1.inverse())).getAngleShortestPath(), 0, 0.01); //These two quaternions are basically the same but expressed on the other side of quaternion space
  btQuaternion q3(-q1.x(), -q1.y(), -q1.z(), -q1.w());
  q3.normalize();
  //  printf("%f %f %f %f,%f %f %f %f\n", q2.x(), q2.y(), q2.z(), q2.w(), q3.x(), q3.y(), q3.z(), q3.w());

  EXPECT_NEAR(q3.angleShortestPath(q1), 0, 0.01); //These two quaternions are basically the same but expressed on the other side of quaternion space
  EXPECT_NEAR((q3*(q1.inverse())).getAngleShortestPath(), 0, 0.01); //These two quaternions are basically the same but expressed on the other side of quaternion space
  
}

TEST(Bullet, AngleQuaternionQuaternionShortestPath)
{
  btQuaternion q1 (SIMD_PI/4,0,0);
  btQuaternion q2(-q1.x(), -q1.y(), -q1.z()-.0001, -q1.w()+.0001);
  q2.normalize();

  //  printf("%f %f %f %f,%f %f %f %f\n", q2.x(), q2.y(), q2.z(), q2.w(), q3.x(), q3.y(), q3.z(), q3.w());

  EXPECT_NEAR(angleShortestPath(q1, q2), 0, 0.01); //These two quaternions are basically the same but expressed on the other side of quaternion space
}

TEST(Bullet, EulerConventionsYPR)
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
  
  btQuaternion q(0,0,0,1);
  btVector3 v(1,0,0);
  q.setEulerZYX(yaw, pitch,roll);
  btTransform tr(q, btVector3(0,0,0));
  
  btVector3 v2 = tr * v;

  printf("%f, %f, %f: yaw by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), yaw, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 1,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 

  btMatrix3x3 mat;
  mat.setEulerZYX(yaw, pitch,roll);
  tr = btTransform(mat, btVector3(0,0,0));
  v2 = tr * v;

  printf("%f, %f, %f: yaw by %f -> %f, %f, %f\n", v.x(), v.y(), v.z(), yaw, v2.x(), v2.y(), v2.z());
  EXPECT_NEAR(v2.x(), 0,epsilon); 
  EXPECT_NEAR(v2.y(), 1,epsilon); 
  EXPECT_NEAR(v2.z(), 0,epsilon); 
                 
  mat.setEulerYPR(yaw, pitch,roll);
  tr = btTransform(mat, btVector3(0,0,0));
  v2 = tr * v;

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

TEST(Bullet, EulerConventionsConstructors)
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

TEST(Bullet, Angle)
{
  seed_rand();
  unsigned int num_steps = 1000;
  double epsilon = 1e-3;
  btQuaternion q1(0, 0, 0, 1);
  for(unsigned int i = 0; i < num_steps; ++i){
    double q1_angle = 1.0 * i / num_steps * 2. * M_PI;
    q1.setRPY(1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX, 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX, q1_angle);
    for(unsigned int j = 0; j < num_steps; j++){
      btQuaternion q2;
      double q2_angle = 1.0 * j / num_steps * 2. * M_PI;
      q2.setRPY(1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX, 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX, q2_angle);
      //EXPECT_NEAR(q2.angle(q1), fabs(q2_angle - q1_angle), epsilon);

      double ratio = (double) rand() / (double) RAND_MAX;
      EXPECT_GE(ratio, 0.0);
      EXPECT_LE(ratio, 1.0);

      btQuaternion q3 = q2.slerp(q1, ratio);
      EXPECT_NEAR(angleShortestPath(q3, q2), angleShortestPath(q2, q1) * ratio, epsilon);
      EXPECT_NEAR(angleShortestPath(q3, q1), angleShortestPath(q2, q1) * (1 - ratio), epsilon);

      btQuaternion q4 = q1.slerp(q2, ratio);
      EXPECT_NEAR(angleShortestPath(q4, q1), angleShortestPath(q2, q1) * ratio, epsilon);
      EXPECT_NEAR(angleShortestPath(q4, q2), angleShortestPath(q2, q1) * (1  - ratio), epsilon);

      /*
      printf("Ratio: %f, q1(%f,%f,%f,%f) q2(%f,%f,%f,%f) q3(%f,%f,%f,%f) q1_norm: %f, q2_norm: %f, q3_norm: %f, q4_norm: %f\n", 
             ratio, q1.x(), q1.y(), q1.z(), q1.w(), q2.x(), q2.y(), q2.z(), q2.w(), q3.x(), q3.y(), q3.z(), q3.w(), q1.length2(), q2.length2(), q3.length2(), q4.length());
      */


      q2.setX(-1.0 * q2.x());
      q2.setY(-1.0 * q2.y());
      q2.setZ(-1.0 * q2.z());
      q2.setW(-1.0 * q2.w());

      //EXPECT_NEAR(q2.angle(q1), fabs(q2_angle - q1_angle), epsilon);
      q3 = q2.slerp(q1, ratio);
      EXPECT_NEAR(angleShortestPath(q3, q2), angleShortestPath(q2, q1) * ratio, epsilon);
      EXPECT_NEAR(angleShortestPath(q3, q1), angleShortestPath(q2, q1) * (1 - ratio), epsilon);

      q4 = q1.slerp(q2, ratio);
      EXPECT_NEAR(angleShortestPath(q4, q1), angleShortestPath(q2, q1) * ratio, epsilon);
      EXPECT_NEAR(angleShortestPath(q4, q2), angleShortestPath(q2, q1) * (1  - ratio), epsilon);

      EXPECT_LT(angleShortestPath(q1, q2), M_PI);
    }
  }

  btQuaternion q5(-0.00285953665482, 0.0134168786627, 0.871697390887, -0.489852497327);
  btQuaternion q6(0.00256516236927, -0.0136650510447, -0.865612832372, 0.500520839481);

  btQuaternion q7 = q6.slerp(q5, 0.5);
  EXPECT_NEAR(angleShortestPath(q7, q5), angleShortestPath(q5, q6) / 2, epsilon);
  EXPECT_NEAR(angleShortestPath(q7, q6), angleShortestPath(q5, q6) / 2, epsilon);

  btQuaternion q8 = q5.slerp(q6, 0.5);
  EXPECT_NEAR(angleShortestPath(q8, q5), angleShortestPath(q5, q6) / 2, epsilon);
  EXPECT_NEAR(angleShortestPath(q8, q6), angleShortestPath(q5, q6) / 2, epsilon);
}


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


TEST(Bullet, SlerpZeroDistanceOppositeSigns)
{
  btQuaternion q1 (M_PI/4,0,0);
  btQuaternion q2(-q1.x(), -q1.y(), -q1.z()-.0001, -q1.w()+.0001);
  q2.normalize();
  btQuaternion q3 = q2.slerp(q1, .5);

  //  printf("%f %f %f %f,%f %f %f %f\n", q2.x(), q2.y(), q2.z(), q2.w(), q3.x(), q3.y(), q3.z(), q3.w());

  EXPECT_NEAR(q1.angleShortestPath(q2), 0, 0.01);
  EXPECT_NEAR(q2.angleShortestPath(q2), 0, 0.01);
  EXPECT_NEAR(q1.angleShortestPath(q3), 0, 0.01);
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
  for (unsigned int i = 1 ; i < 1000 ; i++)
  {
    btQuaternion q1(M_PI*2 *(double) i / 1000, 0, 0);
    btQuaternion q2(M_PI/2*0, 0,0);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q1, q2, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(std::min(M_PI*2 *(double) i / 1000, 2 * M_PI - M_PI*2 *(double) i / 1000), ang, 0.001);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q2, q1, vec, ang);
    //printf("%f %f %f, ang %f %d\n", vec.x(), vec.y(), vec.z(), ang, i);
    if (i <= 500)
      EXPECT_NEAR( vec.z(), 1.0, 0.001);
    else
      EXPECT_NEAR( vec.z(), -1.0, 0.001);

    EXPECT_NEAR(std::min(M_PI*2 *(double) i / 1000, 2 * M_PI - M_PI*2 *(double) i / 1000), ang, 0.001);
    if (i <= 500)
      EXPECT_NEAR( vec.z(), 1.0, 0.001);
    else
      EXPECT_NEAR( vec.z(), -1.0, 0.001);
  }
  for (unsigned int i = 1 ; i < 1000 ; i++)
  {
    btQuaternion q1(0, M_PI*2 *(double) i / 1000,1);
    btQuaternion q2(0, 0, 1);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q1, q2, vec, ang);
    //printf("%f %f %f, ang %f %d\n", vec.x(), vec.y(), vec.z(), ang, i);
    EXPECT_NEAR(std::min(M_PI*2 *(double) i / 1000, 2 * M_PI - M_PI*2 *(double) i / 1000), ang, 0.001);
    if (i < 500)
      EXPECT_NEAR( vec.y(), -1.0, 0.001);
    else if (i > 501)
      EXPECT_NEAR( vec.y(), 1.0, 0.001);
    else
      EXPECT_NEAR( fabs(vec.y()), 1.0, 0.001);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q2, q1, vec, ang);
    //printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang); 
    EXPECT_NEAR(std::min(M_PI*2 *(double) i / 1000, 2 * M_PI - M_PI*2 *(double) i / 1000), ang, 0.001);
    if (i < 500)
      EXPECT_NEAR( vec.y(), 1.0, 0.001);
    else if (i > 500)
      EXPECT_NEAR( vec.y(), -1.0, 0.001);
    else
      EXPECT_NEAR( fabs(vec.y()), 1.0, 0.001);
  }
  for (unsigned int i = 1 ; i < 1000 ; i++)
  {
    btQuaternion q1(0, 0, M_PI*2 *(double) i / 1000);
    btQuaternion q2(0, 0,0);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q1, q2, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(std::min(M_PI*2 *(double) i / 1000, 2 * M_PI - M_PI*2 *(double) i / 1000), ang, 0.001);
    if (i <= 500)
      EXPECT_NEAR( vec.x(), -1.0, 0.001);
    else
      EXPECT_NEAR( vec.x(), 1.0, 0.001);
    btTransformUtil::calculateDiffAxisAngleQuaternion(q2, q1, vec, ang);
    //    printf("%f %f %f, ang %f\n", vec.x(), vec.y(), vec.z(), ang);
    EXPECT_NEAR(std::min(M_PI*2 *(double) i / 1000, 2 * M_PI - M_PI*2 *(double) i / 1000), ang, 0.001);
    if (i <= 500)
      EXPECT_NEAR( vec.x(), 1.0, 0.001);
    else
      EXPECT_NEAR( vec.x(), -1.0, 0.001);
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
