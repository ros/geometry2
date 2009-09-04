import roslib
roslib.load_manifest('tf')
import rostest
import rospy
import numpy
import rostest
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

    def expect_exception(self, func, exception):
       tripped = False
       try:
           func()
       except exception:
           tripped = True
       self.assert_(tripped)

    class SubClass(tf.Transformer):
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
        self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_seconds() == 0)
        for ti in [3, 5, 10, 11, 19, 20, 21]:
            m.header.stamp.secs = ti
            t.setTransform(m)
            self.assert_(t.getLatestCommonTime("THISFRAME", "PARENT").to_seconds() == ti)

        # Verify that getLatestCommonTime with nonexistent frames raises a tf.error
        self.expect_exception(lambda: t.getLatestCommonTime("MANDALAY", "JUPITER"), tf.error)
        self.expect_exception(lambda: t.lookupTransform("MANDALAY", "JUPITER", rospy.Time()), tf.error)

    def test_smoke(self):
        t = tf.Transformer(True, rospy.Duration().from_seconds(10.0))
        self.common(t)

    def test_subclass(self):
        class TransformerSubclass(tf.Transformer):
            def extra(self):
              return 77
        t = TransformerSubclass(True, rospy.Duration.from_seconds(10.0))
        self.assert_(t.extra() == 77)
        self.common(t)
        self.assert_(t.extra() == 77)

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
    if len(sys.argv) == 1:
        rostest.unitrun('tf', 'directed', TestPython)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestPython(sys.argv[1]))
        unittest.TextTestRunner(verbosity=2).run(suite)
