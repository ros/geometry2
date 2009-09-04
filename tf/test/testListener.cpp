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

#include "tf/transform_listener.h"
#include "ros/ros.h"


class testListener 
{
public:

  tf::TransformListener tf;

  //constructor with name
  testListener() 
  {

  };

  ~testListener()
  {

  };

};


int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "listener");

  //Instantiate a local listener
  testListener testListener;

  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  ros::NodeHandle n;
  while(n.ok())
    {
      std::cout << "The current list of frames is:" <<std::endl;
      std::cout << testListener.tf.allFramesAsString()<<std::endl;
      try
      {
        tf::Stamped<tf::Transform> test_transform;
        testListener.tf.lookupTransform("frame1","frame2", ros::Time(), test_transform);
        std::cout << "Success at " << test_transform.stamp_ << std::endl;
      }
      catch(...)
      {
        std::cout << "Failure" << std::endl;

      }
      sleep(1);
    }

  return 0;
};

