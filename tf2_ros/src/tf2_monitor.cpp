

/*
* Copyright (c) 2008, Willow Garage, Inc.
* Copyright (c) 2015, Open Source Robotics Foundation, Inc.
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


#include "tf2_ros/transform_listener.h"
#include <string>
#include <thread>
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

//TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf

class TFMonitor
{
public:
  std::string framea_, frameb_;
  bool using_specific_chain_;
  
  rclcpp::node::Node::SharedPtr node_;
  rclcpp::subscription::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber_tf_, subscriber_tf_message_;
  std::vector<std::string> chain_;
  std::map<std::string, std::string> frame_authority_map;
  std::map<std::string, std::vector<double> > delay_map;
  std::map<std::string, std::vector<double> > authority_map;
  std::map<std::string, std::vector<double> > authority_frequency_map;
  
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;

  tf2_msgs::msg::TFMessage message_;
  std::mutex map_mutex_;
  
  void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    const tf2_msgs::msg::TFMessage& message = *(msg);
    //TODO(tfoote) recover authority info
    std::string authority = "No authority availabil"; //msg_evt.getPublisherName(); // lookup the authority 

    double average_offset = 0;
    std::unique_lock<std::mutex> my_lock(map_mutex_);  
    for (unsigned int i = 0; i < message.transforms.size(); i++)
    {
      frame_authority_map[message.transforms[i].child_frame_id] = authority;

      double offset = tf2::timeToSec(tf2::get_now()) - tf2_ros::timeToSec(message.transforms[i].header.stamp);
      average_offset  += offset;
      
      std::map<std::string, std::vector<double> >::iterator it = delay_map.find(message.transforms[i].child_frame_id);
      if (it == delay_map.end())
      {
        delay_map[message.transforms[i].child_frame_id] = std::vector<double>(1,offset);
      }
      else
      {
        it->second.push_back(offset);
        if (it->second.size() > 1000) 
          it->second.erase(it->second.begin());
      }
      
    } 
    
    average_offset /= std::max((size_t) 1, message.transforms.size());

    //create the authority log
    std::map<std::string, std::vector<double> >::iterator it2 = authority_map.find(authority);
    if (it2 == authority_map.end())
    {
      authority_map[authority] = std::vector<double>(1,average_offset);
    }
    else
    {
      it2->second.push_back(average_offset);
      if (it2->second.size() > 1000) 
        it2->second.erase(it2->second.begin());
    }
    
    //create the authority frequency log
    std::map<std::string, std::vector<double> >::iterator it3 = authority_frequency_map.find(authority);
    if (it3 == authority_frequency_map.end())
    {
      authority_frequency_map[authority] = std::vector<double>(1,tf2::timeToSec(tf2::get_now()));
    }
    else
    {
      it3->second.push_back(tf2::timeToSec(tf2::get_now()));
      if (it3->second.size() > 1000) 
        it3->second.erase(it3->second.begin());
    }
    
  };

  TFMonitor(bool using_specific_chain, std::string framea  = "", std::string frameb = ""):
    framea_(framea), frameb_(frameb),
    using_specific_chain_(using_specific_chain)
  {
    node_ = rclcpp::node::Node::make_shared("tf_monitor");
    tf_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    
    if (using_specific_chain_)
    {
      std::cout << "Waiting for transform chain to become available between "<< framea_ << " and " << frameb_<< " " << std::flush;
      while (rclcpp::ok() && !buffer_.canTransform(framea_, frameb_, tf2::TimePointZero, tf2::Duration(1.0e9)))
        std::cout << "." << std::flush;
      std::cout << std::endl;
     
      try{
        buffer_._chainAsVector(frameb_, tf2::TimePointZero, framea_, tf2::TimePointZero, frameb_, chain_);
      }
      catch(tf2::TransformException& ex){
        ROS_WARN("Transform Exception %s", ex.what());
        return;
      } 

      /*      cout << "Chain currently is:" <<endl;
      for (unsigned int i = 0; i < chain_.size(); i++)
      {
        cout << chain_[i] <<", ";
      }
      cout <<endl;*/
    }

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 100;
    
    std::function<void(const tf2_msgs::msg::TFMessage::SharedPtr)> standard_callback = std::bind(&TFMonitor::callback, this, std::placeholders::_1);
    subscriber_tf_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf", standard_callback, custom_qos_profile);
    std::function<void(const tf2_msgs::msg::TFMessage::SharedPtr)> static_callback = std::bind(&TFMonitor::callback, this, std::placeholders::_1);
    subscriber_tf_message_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf_static", static_callback, custom_qos_profile);

    // subscriber_tf_ = node_->create_subscriber.subscribe<tf::tfMessage>("tf", 100, boost::bind(&TFMonitor::callback, this, _1));
    // subscriber_tf_message_ = node_.subscribe<tf::tfMessage>("tf_message", 100, boost::bind(&TFMonitor::callback, this, _1));
    
  }

  std::string outputFrameInfo(const std::map<std::string, std::vector<double> >::iterator& it, const std::string& frame_authority)
  {
    std::stringstream ss;
    double average_delay = 0;
    double max_delay = 0;
    for (unsigned int i = 0; i < it->second.size(); i++)
    {
      average_delay += it->second[i];
      max_delay = std::max(max_delay, it->second[i]);
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



  while (rclcpp::ok()){
    counter++;
    //    printf("looping %d\n", counter);
    if (using_specific_chain_)
    {
      auto tmp = buffer_.lookupTransform(framea_, frameb_, tf2::TimePointZero);
      double diff = tf2::timeToSec(tf2::get_now()) - tf2_ros::timeToSec(tmp.header.stamp);
      avg_diff = lowpass * diff + (1-lowpass)*avg_diff;
      if (diff > max_diff) max_diff = diff;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
        std::cout <<std::endl<< std::endl<< std::endl<< "RESULTS: for all Frames" <<std::endl;
      std::unique_lock<std::mutex> lock(map_mutex_);  
      std::cout <<std::endl << "Frames:" <<std::endl;
      std::map<std::string, std::vector<double> >::iterator it = delay_map.begin();
      for ( ; it != delay_map.end() ; ++it)
      {
        if (using_specific_chain_ )
        {
          for ( unsigned int i = 0 ; i < chain_.size(); i++)
          {
            if (it->first != chain_[i])
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
        for (unsigned int i = 0; i < it1->second.size(); i++)
        {
          average_delay += it1->second[i];
          max_delay = std::max(max_delay, it1->second[i]);
        }
        average_delay /= it1->second.size();
        double frequency_out = (double)(it2->second.size())/std::max(0.00000001, (it2->second.back() - it2->second.front()));
        //cout << "output" <<&(*it2) <<" " << it2->second.back() <<" " << it2->second.front() <<" " << std::max((size_t)1, it2->second.size()) << " " << frequency_out << endl;
        std::cout << "Node: " <<it1->first << " " << frequency_out <<" Hz, Average Delay: " << average_delay << " Max Delay: " << max_delay << std::endl;
      }
      
    }
  }

  }
};


int main(int argc, char ** argv)
{
  //Initialize ROS
  rclcpp::init(argc, argv);
  
  //TODO(tfoote) make anonymous 
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("tf_monitor_main");


  std::string framea, frameb;
  bool using_specific_chain = true;
  if (argc == 3){
    framea = argv[1];
    frameb = argv[2];
  }
  else if (argc == 1)
    using_specific_chain = false;
  else{
    ROS_INFO("TF_Monitor: usage: tf_monitor framea frameb");
    return -1;
  }
  
  // TODO(tfoote) restore simtime logic
  // //Make sure we don't start before recieving time when in simtime
  // int iterations = 0;
  // while (ros::Time::now() == ros::Time())
  // {
  //   if (++iterations > 10)
  //   {
  //     ROS_INFO("tf_monitor waiting for time to be published");
  //     iterations = 0;
  //   }
  //   ros::WallDuration(0.1).sleep();
  // }

  // This lambda is required because `std::thread` cannot infer the correct
  // rclcpp::spin, since there are more than one versions of it (overloaded).
  // see: http://stackoverflow.com/a/27389714/671658
  // I (wjwwood) chose to use the lamda rather than the static cast solution.
  auto run_func = [](rclcpp::node::Node::SharedPtr node) {
    return rclcpp::spin(node);
  };
  std::thread spinner(run_func, nh);
  TFMonitor monitor(using_specific_chain, framea, frameb);
  monitor.spin();
  spinner.join();
  return 0;

}
