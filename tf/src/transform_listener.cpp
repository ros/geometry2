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

/** \author Tully Foote */

#include "tf/transform_listener.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>


using namespace tf;

std::string tf::remap(const std::string& frame_id)
{
  ros::NodeHandle n;
  std::string tf_prefix;
  n.getParam("~tf_prefix", tf_prefix);
  return tf::remap(tf_prefix, frame_id);
};

/** \brief Convert the transform to a Homogeneous matrix for large operations */
static boost::numeric::ublas::matrix<double> transformAsMatrix(const Transform& bt) 
{
  boost::numeric::ublas::matrix<double> outMat(4,4);

  //  double * mat = outMat.Store();

  double mv[12];
  bt.getBasis().getOpenGLSubMatrix(mv);

  Vector3 origin = bt.getOrigin();

  outMat(0,0)= mv[0];
  outMat(0,1)  = mv[4];
  outMat(0,2)  = mv[8];
  outMat(1,0)  = mv[1];
  outMat(1,1)  = mv[5];
  outMat(1,2)  = mv[9];
  outMat(2,0)  = mv[2];
  outMat(2,1)  = mv[6];
  outMat(2,2) = mv[10];

  outMat(3,0)  = outMat(3,1) = outMat(3,2) = 0;
  outMat(0,3) = origin.x();
  outMat(1,3) = origin.y();
  outMat(2,3) = origin.z();
  outMat(3,3) = 1;


  return outMat;
};

TransformListener::TransformListener(ros::Node & rosnode,
                                     bool interpolating,
                                     ros::Duration max_cache_time):
  Transformer(interpolating, max_cache_time), dedicated_listener_thread_(NULL)
{
  init();
  using_dedicated_thread_ = true; //Node API spins a thread automatically
}

TransformListener::TransformListener(ros::Duration max_cache_time, bool spin_thread):
  Transformer(true, max_cache_time), dedicated_listener_thread_(NULL)
{
  if (spin_thread)
    initWithThread();
  else
    init();
}

TransformListener::TransformListener(const ros::NodeHandle& nh, ros::Duration max_cache_time, bool spin_thread):
  Transformer(true, max_cache_time), dedicated_listener_thread_(NULL), node_(nh)
{
  if (spin_thread)
    initWithThread();
  else
    init();
}

TransformListener::~TransformListener()
{
  using_dedicated_thread_ = false;
  if (dedicated_listener_thread_)
  {
    dedicated_listener_thread_->join();
    delete dedicated_listener_thread_;
  }
}

void TransformListener::init()
{
  message_subscriber_ = node_.subscribe<tf::tfMessage>("/tf_message", 100, boost::bind(&TransformListener::subscription_callback, this, _1)); ///\todo magic number
  
  reset_time_subscriber_ = node_.subscribe<std_msgs::Empty>("/reset_time", 100, boost::bind(&TransformListener::reset_callback, this, _1)); ///\todo magic number
  
  if (! ros::service::exists("~tf_frames", false))  // Avoid doublely advertizing if multiple instances of this library
    {
      tf_frames_srv_ = node_.advertiseService("~tf_frames", &TransformListener::getFrames, this);
    }
    
    node_.param(std::string("~tf_prefix"), tf_prefix_, std::string(""));
}

void TransformListener::initWithThread()
{
  using_dedicated_thread_ = true;
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<tf::tfMessage>("/tf_message", 100, boost::bind(&TransformListener::subscription_callback, this, _1), ros::VoidPtr(), &tf_message_callback_queue_); ///\todo magic number
    
    message_subscriber_ = node_.subscribe(ops);
  
  ros::SubscribeOptions reset_ops = ros::SubscribeOptions::create<std_msgs::Empty>("/reset_time", 100, boost::bind(&TransformListener::reset_callback, this, _1), ros::VoidPtr(), &tf_message_callback_queue_); ///\todo magic number

  reset_time_subscriber_ = node_.subscribe(reset_ops);
  
  dedicated_listener_thread_ = new boost::thread(boost::bind(&TransformListener::dedicatedListenerThread, this));

  if (! ros::service::exists("~tf_frames", false))  // Avoid doublely advertizing if multiple instances of this library
    {
      tf_frames_srv_ = node_.advertiseService("~tf_frames", &TransformListener::getFrames, this);
    }
    
    node_.param(std::string("~tf_prefix"), tf_prefix_, std::string(""));
}

void TransformListener::transformQuaternion(const std::string& target_frame,
    const geometry_msgs::QuaternionStamped& msg_in,
    geometry_msgs::QuaternionStamped& msg_out) const
{
  Stamped<Quaternion> pin, pout;
  quaternionStampedMsgToTF(msg_in, pin);
  transformQuaternion(target_frame, pin, pout);
  quaternionStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformVector(const std::string& target_frame,
    const geometry_msgs::Vector3Stamped& msg_in,
    geometry_msgs::Vector3Stamped& msg_out) const
{
  Stamped<Vector3> pin, pout;
  vector3StampedMsgToTF(msg_in, pin);
  transformVector(target_frame, pin, pout);
  vector3StampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPoint(const std::string& target_frame,
    const geometry_msgs::PointStamped& msg_in,
    geometry_msgs::PointStamped& msg_out) const
{
  Stamped<Point> pin, pout;
  pointStampedMsgToTF(msg_in, pin);
  transformPoint(target_frame, pin, pout);
  pointStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPose(const std::string& target_frame,
    const geometry_msgs::PoseStamped& msg_in,
    geometry_msgs::PoseStamped& msg_out) const
{
  Stamped<Pose> pin, pout;
  poseStampedMsgToTF(msg_in, pin);
  transformPose(target_frame, pin, pout);
  poseStampedTFToMsg(pout, msg_out);
}
void TransformListener::transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::QuaternionStamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::QuaternionStamped& msg_out) const
{
  Stamped<Quaternion> pin, pout;
  quaternionStampedMsgToTF(msg_in, pin);
  transformQuaternion(target_frame, target_time, pin, fixed_frame, pout);
  quaternionStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformVector(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::Vector3Stamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::Vector3Stamped& msg_out) const
{
  Stamped<Vector3> pin, pout;
  vector3StampedMsgToTF(msg_in, pin);
  transformVector(target_frame, target_time, pin, fixed_frame, pout);
  vector3StampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPoint(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::PointStamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::PointStamped& msg_out) const
{
  Stamped<Point> pin, pout;
  pointStampedMsgToTF(msg_in, pin);
  transformPoint(target_frame, target_time, pin, fixed_frame, pout);
  pointStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPose(const std::string& target_frame, const ros::Time& target_time,
    const geometry_msgs::PoseStamped& msg_in,
    const std::string& fixed_frame, geometry_msgs::PoseStamped& msg_out) const
{
  Stamped<Pose> pin, pout;
  poseStampedMsgToTF(msg_in, pin);
  transformPose(target_frame, target_time, pin, fixed_frame, pout);
  poseStampedTFToMsg(pout, msg_out);
}

void TransformListener::transformPointCloud(const std::string & target_frame, const sensor_msgs::PointCloud & cloudIn, sensor_msgs::PointCloud & cloudOut) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, cloudIn.header.frame_id, cloudIn.header.stamp, transform);

  transformPointCloud(target_frame, transform, cloudIn.header.stamp, cloudIn, cloudOut);
}
void TransformListener::transformPointCloud(const std::string& target_frame, const ros::Time& target_time, 
    const sensor_msgs::PointCloud& cloudIn,
    const std::string& fixed_frame, sensor_msgs::PointCloud& cloudOut) const
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
      cloudIn.header.frame_id, cloudIn.header.stamp,
      fixed_frame,
      transform);

  transformPointCloud(target_frame, transform, target_time, cloudIn, cloudOut);


}


void TransformListener::transformPointCloud(const std::string & target_frame, const Transform& net_transform, 
                                            const ros::Time& target_time, const sensor_msgs::PointCloud & cloudIn, sensor_msgs::PointCloud & cloudOut) const
{
  boost::numeric::ublas::matrix<double> transform = transformAsMatrix(net_transform);

  unsigned int length = cloudIn.get_points_size();

  boost::numeric::ublas::matrix<double> matIn(4, length);

  double * matrixPtr = matIn.data().begin();

  for (unsigned int i = 0; i < length ; i++)
  {
    matrixPtr[i] = cloudIn.points[i].x;
    matrixPtr[i+length] = cloudIn.points[i].y;
    matrixPtr[i+ 2* length] = cloudIn.points[i].z;
    matrixPtr[i+ 3* length] = 1;
  };

  boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);

  // Copy relevant data from cloudIn, if needed
  if (&cloudIn != &cloudOut)
  {
    cloudOut.header = cloudIn.header;
    cloudOut.set_points_size(length);
    cloudOut.set_channels_size(cloudIn.get_channels_size());
    for (unsigned int i = 0 ; i < cloudIn.get_channels_size() ; ++i)
      cloudOut.channels[i] = cloudIn.channels[i];
  }

  matrixPtr = matOut.data().begin();

  //Override the positions
  cloudOut.header.stamp = target_time;
  cloudOut.header.frame_id = target_frame;
  for (unsigned int i = 0; i < length ; i++)
  {
    cloudOut.points[i].x = matrixPtr[i];
    cloudOut.points[i].y = matrixPtr[i + length];
    cloudOut.points[i].z = matrixPtr[i + 2* length];
  };
}




void TransformListener::subscription_callback(const tf::tfMessageConstPtr& msg)
{
  const tf::tfMessage& msg_in = *msg;
  for (unsigned int i = 0; i < msg_in.transforms.size(); i++)
  {
    Stamped<Transform> trans;
    transformStampedMsgToTF(msg_in.transforms[i], trans);
    try
    {
      std::map<std::string, std::string>* msg_header_map = msg_in.__connection_header.get();
      std::string authority;
      std::map<std::string, std::string>::iterator it = msg_header_map->find("callerid");
      if (it == msg_header_map->end())
      {
        ROS_WARN("Message recieved without callerid");
        authority = "no callerid";
      }
      else 
      {
        authority = it->second;
      }
      setTransform(trans, authority);
    }

    catch (TransformException& ex)
    {
      ///\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", msg_in.transforms[i].child_frame_id.c_str(), msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
};

void TransformListener::reset_callback(const std_msgs::EmptyConstPtr &msg)
{
  clear();
};



