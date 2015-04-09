/*
 * Copyright (c) Koji Terada
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

/** \author Wim Meeussen */


#include <tf2_eigen/tf2_eigen.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf2/convert.h>

static const double EPS = 1e-3;


TEST(TfEigen, ConvertVector)
{
  Eigen::Vector3d v(1,2,3);

  Eigen::Vector3d v1 = v;
  tf2::convert(v1, v1);

  EXPECT_EQ(v, v1);

  Eigen::Vector3d v2(3,4,5);
  tf2::convert(v1, v2);

  EXPECT_EQ(v, v2);
  EXPECT_EQ(v1, v2);
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  bool ret = RUN_ALL_TESTS();
  return ret;
}
