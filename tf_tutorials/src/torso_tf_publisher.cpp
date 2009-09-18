#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;
  while(n.ok()){
    broadcaster.sendTransform(btTransform(btQuaternion(0,0,0),
    btVector3(0.0,0.0,0.0)),
    ros::Time::now(),"torso_lift_link", "base_link");  
    r.sleep();
  }
}
