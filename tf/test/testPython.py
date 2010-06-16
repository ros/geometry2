import roslib
roslib.load_manifest('tf')
import rostest
import rospy
import numpy
import unittest
import sys

import tf.transformations
import geometry_msgs.msg

import tf

class Mock:
  pass

def setT(t, parent, frame, ti, x):
  m = Mock()
  m.parent_id = parent
  m.header = Mock()
  m.header.stamp = ti
  m.header.frame_id = frame
  m.transform = Mock()
  m.transform.translation = Mock()
  m.transform.translation.x = x
  m.transform.translation.y = 0
  m.transform.translation.z = 0
  m.transform.rotation = Mock()
  m.transform.rotation.x = 0
  m.transform.rotation.y = 0
  m.transform.rotation.z = 0
  m.transform.rotation.w = 1
  t.setTransform(m)

class TestPython(unittest.TestCase):

    def setUp(self):
        pass

    def common(self, t):
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = "PARENT"
        m.child_frame_id = "THISFRAME"
        m.transform.translation.y = 5.0
        m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        t.setTransform(m)
        afs = t.allFramesAsString()
        self.assert_(len(afs) != 0)
        self.assert_("PARENT" in afs)
        self.assert_("THISFRAME" in afs)
        self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == 0)
        for ti in [3, 5, 10, 11, 19, 20, 21]:
            m.header.stamp.secs = ti
            t.setTransform(m)
            self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == ti)

        # Verify that getLatestCommonTime with nonexistent frames raise exception 
        self.assertRaises(tf.Exception, lambda: t.getLatestCommonTime("MANDALAY", "JUPITER"))
        self.assertRaises(tf.LookupException, lambda: t.lookupTransform("MANDALAY", "JUPITER", rospy.Time()))

        # Ask for transform for valid frames, but more than 10 seconds in the past.  Should raise ExtrapolationException
        self.assertRaises(tf.ExtrapolationException, lambda: t.lookupTransform("THISFRAME", "PARENT", rospy.Time(2)))

        #### print t.lookupVelocity("THISFRAME", "PARENT", rospy.Time(15), rospy.Duration(5))

    def test_smoke(self):
        t = tf.Transformer()
        self.common(t)

    def test_cache_time(self):
        # Vary cache_time and confirm its effect on ExtrapolationException from lookupTransform().

        for cache_time in range(2, 98):
            t = tf.Transformer(True, rospy.Duration(cache_time))
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "PARENT"
            m.child_frame_id = "THISFRAME"
            m.transform.translation.y = 5.0
            m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
            t.setTransform(m)
            afs = t.allFramesAsString()
            self.assert_(len(afs) != 0)
            self.assert_("PARENT" in afs)
            self.assert_("THISFRAME" in afs)
            self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == 0)

            # Set transforms for time 0..100 inclusive
            for ti in range(101):
                m.header.stamp = rospy.Time(ti)
                t.setTransform(m)
                self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec() == ti)
            self.assertEqual(t.getLatestCommonTime("THISFRAME", "PARENT").to_sec(), 100)

            # (avoid time of 0 because that means 'latest')

            for ti in range(1, 100 - cache_time):
                self.assertRaises(tf.ExtrapolationException, lambda: t.lookupTransform("THISFRAME", "PARENT", rospy.Time(ti)))
            for ti in range(100 - cache_time, 100):
                t.lookupTransform("THISFRAME", "PARENT", rospy.Time(ti))

    def test_subclass(self):
        class TransformerSubclass(tf.Transformer):
            def extra(self):
              return 77
        t = TransformerSubclass(True, rospy.Duration.from_seconds(10.0))
        self.assert_(t.extra() == 77)
        self.common(t)
        self.assert_(t.extra() == 77)

    def test_twist(self):
        t = tf.Transformer()

        vel = 3
        for ti in range(5):
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = "PARENT"
            m.header.stamp = rospy.Time(ti)
            m.child_frame_id = "THISFRAME"
            m.transform.translation.x = ti * vel
            m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
            t.setTransform(m)

        tw0 = t.lookupTwist("THISFRAME", "PARENT", rospy.Time(0.0), rospy.Duration(4.001))
        self.assertAlmostEqual(tw0[0][0], vel, 2)
        tw1 = t.lookupTwistFull("THISFRAME", "PARENT", "PARENT", (0, 0, 0), "THISFRAME", rospy.Time(0.0), rospy.Duration(4.001))
        self.assertEqual(tw0, tw1)

    def test_transformer_ros(self):
        tr = tf.TransformerROS()
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = "PARENT"
        m.child_frame_id = "THISFRAME"
        m.transform.translation.y = 5.0
        m.transform.rotation.x = 0.04997917
        m.transform.rotation.y = 0
        m.transform.rotation.z = 0
        m.transform.rotation.w = 0.99875026
        tr.setTransform(m)

        # Smoke the various transform* methods

        types = [ "Point", "Pose", "Quaternion", "Vector3" ]
        for t in types:
            msg = getattr(geometry_msgs.msg, "%sStamped" % t)()
            msg.header.frame_id = "THISFRAME"
            msg_t = getattr(tr, "transform%s" % t)("PARENT", msg)
            self.assertEqual(msg_t.header.frame_id, "PARENT")


        """
        Two fixed quaternions, a small twist around X concatenated.

        >>> t.quaternion_from_euler(0.1, 0, 0)
        array([ 0.04997917,  0.        ,  0.        ,  0.99875026])
        >>> t.quaternion_from_euler(0.2, 0, 0)
        array([ 0.09983342,  0.        ,  0.        ,  0.99500417])
        """

        # Specific test for quaternion types

        msg = geometry_msgs.msg.QuaternionStamped()
        q = [ 0.04997917,  0.        ,  0.        ,  0.99875026 ]
        msg.quaternion.x = q[0]
        msg.quaternion.y = q[1]
        msg.quaternion.z = q[2]
        msg.quaternion.w = q[3]
        msg.header.frame_id = "THISFRAME"
        msg_t = tr.transformQuaternion("PARENT", msg)
        self.assertEqual(msg_t.header.frame_id, "PARENT")
        for a,v in zip("xyzw", [ 0.09983342,  0.        ,  0.        ,  0.99500417]):
            self.assertAlmostEqual(v,
                                   getattr(msg_t.quaternion, a),
                                   4)
    def test_getTFPrefix(self):
        t = tf.Transformer()
        self.assertEqual(t.getTFPrefix(), "")

    def no_test_random(self):
        import networkx as nx
        for (r,h) in [ (2,2), (2,5), (3,5) ]:
            G = nx.balanced_tree(r, h)
            t = tf.Transformer(True, rospy.Duration(10.0))

            for n in G.nodes():
                if n != 0:
                    # n has parent p
                    p = min(G.neighbors(n))
                    setT(t, str(p), str(n), rospy.Time(0), 1)
            for n in G.nodes():
                ((x,_,_), _) = t.lookupTransform("0", str(n), rospy.Time(0))
                self.assert_(x == nx.shortest_path_length(G, 0, n))
            for i in G.nodes():
                for j in G.nodes():
                    ((x,_,_), _) = t.lookupTransform(str(i), str(j), rospy.Time())
                    self.assert_(abs(x) == abs(nx.shortest_path_length(G, 0, i) - nx.shortest_path_length(G, 0, j)))

if __name__ == '__main__':
    if len(sys.argv) == 1 or sys.argv[1].startswith('--gtest_output'):
        rostest.unitrun('tf', 'directed', TestPython)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestPython(sys.argv[1]))
        unittest.TextTestRunner(verbosity=2).run(suite)
