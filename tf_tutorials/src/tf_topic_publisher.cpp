#include "ros/ros.h"
#include "ros/node.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc,argv,"message_filter_tester");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/object",1);

  geometry_msgs::PoseStamped test;
  test.pose.position.x = 0.0;
  test.pose.position.y = 0.0;
  test.pose.position.z = 0.0;
  test.pose.orientation.x = 0.0;
  test.pose.orientation.y = 0.0;
  test.pose.orientation.z = 0.0;
  test.pose.orientation.w = 1.0;
  test.header.frame_id = "base_link";

  ros::Duration st(2.0);
  while(ros::ok())
  {
    test.header.stamp = ros::Time::now()+ros::Duration(1.0);
    pub.publish(test);
    ROS_INFO("Published cmd with timestamp: %f and x position: %f",test.header.stamp.toSec(),test.pose.position.x);
    test.pose.position.x += 1.0;
    st.sleep();
  }
  return 0;
}
