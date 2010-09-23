#!/usr/bin/python

import roslib; roslib.load_manifest('tf2_kdl')
import rospy
import PyKDL
import tf2_py
import tf2_kdl

tf2_py.TransformRegistration().print_me()

def main():
    b = tf2_py.Buffer()
    print b.lookupTransform('a','b', rospy.Time(0), rospy.Duration(2.0))
    v = PyKDL.Vector(1,2,3)
    print b.transform(tf2_py.Stamped(v, 'a', rospy.Time(2)), 'b')



if __name__ == '__main__':
    rospy.init_node('wim')
    main()
