#!/usr/bin/python

import roslib; roslib.load_manifest('tf2_kdl')
import rospy
import PyKDL
import tf2_py
import tf2_kdl
from geometry_msgs.msg import TransformStamped

def main():
    b = tf2_py.Buffer()
    t = TransformStamped()
    t.transform.translation.x = 1
    t.transform.rotation.x = 1
    t.header.stamp = rospy.Time(2.0)
    t.header.frame_id = 'a'
    t.child_frame_id = 'b'
    b.setTransform(t, 'eitan_rocks')
    print b.lookupTransform('a','b', rospy.Time(2.0), rospy.Duration(2.0))

    v = PyKDL.Vector(1,2,3)
    print b.transform(tf2_py.Stamped(v, 'a', rospy.Time(2)), 'b')

    f = PyKDL.Frame(PyKDL.Rotation.RPY(1,2,3), PyKDL.Vector(1,2,3))
    print b.transform(tf2_py.Stamped(f, 'a', rospy.Time(2)), 'b')

    t = PyKDL.Twist(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
    print b.transform(tf2_py.Stamped(t, 'a', rospy.Time(2)), 'b')

    w = PyKDL.Wrench(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
    print b.transform(tf2_py.Stamped(w, 'a', rospy.Time(2)), 'b')


if __name__ == '__main__':
    rospy.init_node('wim')
    main()
