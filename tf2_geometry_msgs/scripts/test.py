#!/usr/bin/python

import roslib; roslib.load_manifest('tf2_geometry_msgs')
import rospy
import PyKDL
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped, Vector3Stamped, PoseStamped


def main():
    b = tf2_ros.Buffer()
    t = TransformStamped()
    t.transform.translation.x = 1
    t.transform.rotation.x = 1
    t.header.stamp = rospy.Time(2.0)
    t.header.frame_id = 'a'
    t.child_frame_id = 'b'
    b.set_transform(t, 'eitan_rocks')
    print b.lookup_transform('a','b', rospy.Time(2.0), rospy.Duration(2.0))

    v = PointStamped()
    v.header.stamp = rospy.Time(2)
    v.header.frame_id = 'a'
    v.point.x = 1
    v.point.y = 2
    v.point.z = 3
    print b.transform(v, 'b')

    v = Vector3Stamped()
    v.header.stamp = rospy.Time(2)
    v.header.frame_id = 'a'
    v.vector.x = 1
    v.vector.y = 2
    v.vector.z = 3
    print b.transform(v, 'b')

    v = PoseStamped()
    v.header.stamp = rospy.Time(2)
    v.header.frame_id = 'a'
    v.pose.position.x = 1
    v.pose.position.y = 2
    v.pose.position.z = 3
    v.pose.orientation.x = 1
    print b.transform(v, 'b')

if __name__ == '__main__':
    rospy.init_node('wim')
    main()
