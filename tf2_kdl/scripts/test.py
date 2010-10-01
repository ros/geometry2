#!/usr/bin/python

import roslib; roslib.load_manifest('tf2_kdl')
import rospy
import PyKDL
import tf2_ros
import tf2_kdl
from geometry_msgs.msg import TransformStamped
from copy import deepcopy

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

    v = PyKDL.Vector(1,2,3)
    print b.transform(tf2_ros.Stamped(v, rospy.Time(2), 'a'), 'b')

    f = PyKDL.Frame(PyKDL.Rotation.RPY(1,2,3), PyKDL.Vector(1,2,3))
    print b.transform(tf2_ros.Stamped(f, rospy.Time(2), 'a'), 'b')

    t = PyKDL.Twist(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
    print b.transform(tf2_ros.Stamped(t, rospy.Time(2), 'a'), 'b')

    w = PyKDL.Wrench(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
    print b.transform(tf2_ros.Stamped(w, rospy.Time(2), 'a'), 'b')

def convert():
    v = PyKDL.Vector(1,2,3)
    vs = tf2_ros.Stamped(v, rospy.Time(2), 'a')
    vs2 = tf2_ros.convert(vs, PyKDL.Vector)
    vs2[1] = 100
    print vs2
    print v


if __name__ == '__main__':
    rospy.init_node('wim')
    main()
    convert()
