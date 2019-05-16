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

/** \author Wim Meeussen */


#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"

class TFMonitor
{
public:
  std::string framea_, frameb_;
  bool using_specific_chain_;

  ros::NodeHandle node_;
  std::vector<std::string> chain_;
  std::map<std::string, std::string> frame_authority_map;
  std::map<std::string, std::vector<double> > delay_map;
  std::map<std::string, std::vector<double> > authority_map;
  std::map<std::string, std::vector<double> > authority_frequency_map;

  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  tf2_msgs::TFMessage message_;

  boost::mutex map_lock_;
  void callback(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt)
  {
    const tf2_msgs::TFMessage& message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName(); // lookup the authority
    process_callback(message, authority, false);
  }

  void static_callback(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt)
  {
    const tf2_msgs::TFMessage& message = *(msg_evt.getConstMessage());
    std::string authority = msg_evt.getPublisherName() + std::string(" (static)"); // lookup the authority
    process_callback(message, authority, true);
  }


  void process_callback(const tf2_msgs::TFMessage& message, const std::string& authority, bool is_static)
  {
    double average_offset = 0;
    boost::mutex::scoped_lock my_lock(map_lock_);
    for (const auto& transform : message.transforms)
    {
      frame_authority_map[transform.child_frame_id] = authority;

      double offset;
      if (is_static)
      {
        offset = 0.0;
      }
      else
      {
        offset = (ros::Time::now() - transform.header.stamp).toSec();
      }
      average_offset += offset;

      std::map<std::string, std::vector<double> >::iterator it = delay_map.find(transform.child_frame_id);
      if (it == delay_map.end())
      {
        delay_map[transform.child_frame_id] = std::vector<double>(1, offset);
      }
      else
      {
        it->second.push_back(offset);
        if (it->second.size() > 1000)
        {
          it->second.erase(it->second.begin());
        }
      }

    }

    average_offset /= std::max((size_t) 1, message.transforms.size());

    // create the authority log
    std::map<std::string, std::vector<double> >::iterator it2 = authority_map.find(authority);
    if (it2 == authority_map.end())
    {
      authority_map[authority] = std::vector<double>(1, average_offset);
    }
    else
    {
      it2->second.push_back(average_offset);
      if (it2->second.size() > 1000)
      {
        it2->second.erase(it2->second.begin());
      }
    }

    // create the authority frequency log
    std::map<std::string, std::vector<double> >::iterator it3 = authority_frequency_map.find(authority);
    if (it3 == authority_frequency_map.end())
    {
      authority_frequency_map[authority] = std::vector<double>(1,ros::Time::now().toSec());
    }
    else
    {
      it3->second.push_back(ros::Time::now().toSec());
      if (it3->second.size() > 1000)
        it3->second.erase(it3->second.begin());
    }

  };

  TFMonitor(bool using_specific_chain, std::string framea  = "", std::string frameb = ""):
    framea_(framea), frameb_(frameb),
    using_specific_chain_(using_specific_chain),
    tf_listener_(tf_buffer_)
  {

    if (using_specific_chain_)
    {
      std::cout << "Waiting for transform chain to become available between "<< framea_ << " and " << frameb_<< "..." << std::flush;
      while (node_.ok() && !tf_buffer_.canTransform(framea_, frameb_, ros::Time()));
      std::cout << std::endl;

      try
      {
        tf_buffer_._chainAsVector(frameb_, ros::Time(), framea_, ros::Time(), frameb_, chain_);
      }
      catch(tf2::TransformException& ex)
      {
        ROS_WARN("Transform Exception %s", ex.what());
        return;
      }

    }

  }

  std::string outputFrameInfo(const std::map<std::string, std::vector<double> >::iterator& it, const std::string& frame_authority)
  {
    std::stringstream ss;
    double average_delay = 0;
    double max_delay = 0;
    for (double i : it->second)
    {
      average_delay += i;
      max_delay = std::max(max_delay, i);
    }
    average_delay /= it->second.size();
    ss << "Frame: " << it->first <<" published by "<< frame_authority << " Average Delay: " << average_delay << " Max Delay: " << max_delay << std::endl;
    return ss.str();
  }

  void spin()
  {
    // create tf listener
    double max_diff = 0;
    double avg_diff = 0;
    double lowpass = 0.01;
    unsigned int counter = 0;

    while (node_.ok()){
      geometry_msgs::TransformStamped tmp;
      counter++;
      if (using_specific_chain_)
      {
        tmp = tf_buffer_.lookupTransform(framea_, frameb_, ros::Time());
        double diff = (ros::Time::now() - tmp.header.stamp).toSec();
        avg_diff = lowpass * diff + (1-lowpass)*avg_diff;
        if (diff > max_diff) max_diff = diff;
      }
      ros::Duration(0.01).sleep();
      if (counter > 20){
        counter = 0;

        if (using_specific_chain_)
        {
          std::cout <<std::endl<< std::endl<< std::endl<< "RESULTS: for "<< framea_ << " to " << frameb_  <<std::endl;
          std::cout << "Chain is: ";
          for (unsigned int i = 0; i < chain_.size(); i++)
          {
            std::cout << chain_[i] ;
            if (i != chain_.size()-1)
              std::cout <<" -> ";
          }
          std::cout << std::endl;
          std::cout << "Net delay " << "    avg = " << avg_diff <<": max = " << max_diff << std::endl;
        }
        else
        {
            std::cout << std::endl << std::endl << std::endl << "RESULTS: for all Frames" << std::endl;
        }
        boost::mutex::scoped_lock lock(map_lock_);
        std::cout <<std::endl << "Frames:" <<std::endl;
        std::map<std::string, std::vector<double> >::iterator it = delay_map.begin();
        for ( ; it != delay_map.end() ; ++it)
        {
          if (using_specific_chain_ )
          {
            for (const auto & i : chain_)
            {
              if (it->first != i)
                continue;

              std::cout << outputFrameInfo(it, frame_authority_map[it->first]);
            }
          }
          else
            std::cout << outputFrameInfo(it, frame_authority_map[it->first]);
        }
        std::cerr <<std::endl<< "All Broadcasters:" << std::endl;
        std::map<std::string, std::vector<double> >::iterator it1 = authority_map.begin();
        std::map<std::string, std::vector<double> >::iterator it2 = authority_frequency_map.begin();
        for ( ; it1 != authority_map.end() ; ++it1, ++it2)
        {
          double average_delay = 0;
          double max_delay = 0;
          for (double i : it1->second)
          {
            average_delay += i;
            max_delay = std::max(max_delay, i);
          }
          average_delay /= it1->second.size();
          double frequency_out = (double)(it2->second.size())/std::max(0.00000001, (it2->second.back() - it2->second.front()));
          std::cout << "Node: " <<it1->first << " " << frequency_out <<" Hz, Average Delay: " << average_delay << " Max Delay: " << max_delay << std::endl;
        }

      }
    }

  }
};


int main(int argc, char ** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "tf_monitor", ros::init_options::AnonymousName);


  std::string framea, frameb;
  bool using_specific_chain = true;
  if (argc == 3) {
    framea = argv[1];
    frameb = argv[2];
  }
  else if (argc == 1) {
    using_specific_chain = false;
  } else {
    ROS_INFO("TF_Monitor: usage: tf_monitor framea frameb");
    return -1;
  }

  ros::NodeHandle nh;

  // Make sure we don't start before receiving time when in simtime
  int iterations = 0;
  while (ros::Time::now() == ros::Time()) {
    if (++iterations > 10) {
      ROS_INFO("tf_monitor waiting for time to be published");
      iterations = 0;
    }
    ros::WallDuration(0.1).sleep();
  }

  boost::thread spinner( boost::bind( &ros::spin ));
  TFMonitor monitor(using_specific_chain, framea, frameb);
  ros::Subscriber subscriber_tf_ = nh.subscribe("tf", 100, &TFMonitor::callback, &monitor);
  ros::Subscriber subscriber_tf_static_ = nh.subscribe("tf_static", 100, &TFMonitor::static_callback, &monitor);
  monitor.spin();
  spinner.join();
  return 0;

}
