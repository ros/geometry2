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

#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/node.h"

class echoListener
{
public:

  tf::TransformListener tf;

  //constructor with name
  echoListener(ros::Node& anode):
    tf(anode),
    node_(anode)
  {

  };

  ~echoListener()
  {

  };

  bool ok(){return node_.ok();};
private:
  ros::Node& node_;

};


int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv);

  if (argc != 3)
  {
    printf("Usage: tf_echo source_frame target_frame\n");
    return -1;
  }


  ros::Node node("tf_echo", ros::Node::ANONYMOUS_NAME);
  //Instantiate a local listener
  echoListener echoListener(node);


  std::string source_frameid = std::string(argv[1]);
  std::string target_frameid = std::string(argv[2]);


  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while(echoListener.ok())
    {
      try
      {
        tf::Stamped<tf::Transform> echo_transform;
        echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
        std::cout << "Success at " << echo_transform.stamp_ << std::endl;
        double yaw, pitch, roll;
        echo_transform.getBasis().getEulerZYX(yaw, pitch, roll);
        tf::Quaternion q = echo_transform.getRotation();
        tf::Vector3 v = echo_transform.getOrigin();
        printf("[%f %f %f %f] Euler(%f %f %f)\n", q.getX(), q.getY(), q.getZ(), q.getW(), yaw, pitch, roll);
        printf("Translation: [%f %f %f]\n", v.getX(), v.getY(), v.getZ());

        //print transform
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << echoListener.tf.allFramesAsString()<<std::endl;
        
      }
      sleep(1);
    }

  return 0;
};

