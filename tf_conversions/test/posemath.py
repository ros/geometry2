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

    def pose_equal(self, a, b):
        ma = a.asMessage()
        mb = b.asMessage()
        self.assertAlmostEqual(ma.position.x, mb.position.x, 3)
        self.assertAlmostEqual(ma.position.y, mb.position.y, 3)
        self.assertAlmostEqual(ma.position.z, mb.position.z, 3)
        self.assertAlmostEqual(ma.orientation.x, mb.orientation.x, 3)
        self.assertAlmostEqual(ma.orientation.y, mb.orientation.y, 3)
        self.assertAlmostEqual(ma.orientation.z, mb.orientation.z, 3)
        self.assertAlmostEqual(ma.orientation.w, mb.orientation.w, 3)

    def test_roundtrip(self):
            
        c = PoseMath.fromEuler(1.0, 3.0 , 7.0,  0, pi/2, 0)
        # Attempt various round-trips c->d, and assert that c == d

        d = PoseMath(c.asMessage())
        self.pose_equal(c, d)

        d = PoseMath.fromMatrix(c.asMatrix())
        self.pose_equal(c, d)

        d = PoseMath.fromEuler(*c.asEuler())
        self.pose_equal(c, d)

        d = PoseMath.fromTf(c.asTf())
        self.pose_equal(c, d)

if __name__ == '__main__':
    if len(sys.argv) == 1 or sys.argv[1].startswith('--gtest_output'):
        rostest.unitrun('tf', 'directed', TestPoseMath)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestPoseMath(sys.argv[1]))
        unittest.TextTestRunner(verbosity=2).run(suite)
