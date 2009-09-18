#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  double dT = 0.01;
  ros::Rate r(1/dT);


  tf::TransformBroadcaster broadcaster;

  double base_odom_distance   = 0.0;
  double base_ball_distance_x = 1.0;
  double base_ball_distance_y = 0.0;

  double ball_x_velocity = 0.0;
  double ball_y_velocity = 0.1;

  double base_velocity = -0.1;

  ros::Time start_time = ros::Time::now();
  while(n.ok()){
    broadcaster.sendTransform(btTransform(btQuaternion(0, 0, 0), 
    btVector3(base_ball_distance_x,
    base_ball_distance_y, 0.0)),
    ros::Time::now(),"ball", "base_link");
    broadcaster.sendTransform(btTransform(btQuaternion(0, 0, 0), 
    btVector3(base_odom_distance, 
    0.0, 0.0)),
    ros::Time::now(),"base_link", 
    "odom");

    if(ros::Time::now()-start_time > ros::Duration(0.1))
    {
      ROS_INFO("ball position in base link is: (%f,%f,%f) m at time: %f",
      base_ball_distance_x,base_ball_distance_y,0.0,ros::Time::now().toSec());
    }

    base_ball_distance_x -= base_velocity*dT;
    base_ball_distance_y += ball_y_velocity*dT;
    base_odom_distance += base_velocity*dT;
    r.sleep();
  }
}
