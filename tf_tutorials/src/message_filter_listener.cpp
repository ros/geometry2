#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <message_filters/subscriber.h>
#include <boost/bind.hpp>
#include "ros/ros.h"

class FilterUsingClass
{
public:
  tf::TransformListener& tfl_;

  void callback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    tf::Stamped<tf::Pose> pose;
    tf::poseStampedMsgToTF(*message, pose);
    ROS_INFO("Notifer Callback");
   try{
      tfl_.transformPose("torso_lift_link", pose, pose);
      ROS_INFO("Received MessageFilter callback at time: %f with x position %f",ros::Time::now().toSec(),pose.getOrigin().x());
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("This should never be called for the notifier only calls back when this is possible. But it's good practice to use it anyway.");
    }
  };
  FilterUsingClass(tf::TransformListener& tfl): tfl_(tfl)
  {
  };
};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc,argv,"message_filter_tutorial");
  ros::NodeHandle nh;
  tf::TransformListener tfl(ros::Duration(15.0));  //Create a TransformListener with 15 seconds of caching
  ROS_INFO("This class will buffer messages coming in on topic objects and provide a callback when they can be transformed into target frame torso lift link");
  FilterUsingClass filter_callback_class(tfl);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub(nh,"object", 100);
  tf::MessageFilter<geometry_msgs::PoseStamped> filter(sub,tfl,"torso_lift_link",100); //queue size
  filter.registerCallback(boost::bind(&FilterUsingClass::callback, &filter_callback_class, _1));

  ros::spin();
  return 0;
}
