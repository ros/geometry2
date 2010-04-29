import roslib
roslib.load_manifest('tf_conversions')
import rostest
import rospy
import numpy
import unittest
import sys

from tf import Transformer
from tf_conversions.posemath import PoseMath

from geometry_msgs.msg import TransformStamped
from math import pi

class TestPoseMath(unittest.TestCase):

    def setUp(self):
        pass

    def test_fromEuler(self):
        a = PoseMath.fromEuler(5,0,0,  0, pi/2, 0)
        self.assert_(repr(a) != "")

    def test_fromTf(self):
        transformer = Transformer(True, rospy.Duration(10.0))
        m = TransformStamped()
        m.header.frame_id = 'wim'
        m.child_frame_id = 'james'
        m.transform.translation.x = 2.71828183
        m.transform.rotation.w = 1.0
        transformer.setTransform(m)
        b = PoseMath.fromTf(transformer.lookupTransform('wim', 'james', rospy.Time(0)))
        
    def test_Operator(self):
        a = PoseMath.fromEuler(5.0, 0.0, 0.0,  0, pi/2, 0)
        b = PoseMath.fromEuler(1.0, 3.0, 7.0,  pi/2, pi/4, pi/3)
        # Exercise the operators
        c = ~b * a
        c = ~(b * ~a)
        c = ~(a * ~b)
        c = b * a

    def test_roundtrip(self):
        c = PoseMath.fromEuler(1.0, 3.0 , 7.0,  0, pi/2, 0)
        # Attempt various round-trips c->d, and assert that c == d

        d = PoseMath(c.asMessage())
        self.assertEqual(repr(c), repr(d))

        d = PoseMath.fromMatrix(c.asMatrix())
        self.assertEqual(repr(c), repr(d))

        d = PoseMath.fromEuler(*c.asEuler())
        # self.assertEqual(repr(c), repr(d))

        d = PoseMath.fromTf(c.asTf())
        self.assertEqual(repr(c), repr(d))

if __name__ == '__main__':
    if len(sys.argv) == 1 or sys.argv[1].startswith('--gtest_output'):
        rostest.unitrun('tf', 'directed', TestPoseMath)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestPoseMath(sys.argv[1]))
        unittest.TextTestRunner(verbosity=2).run(suite)
