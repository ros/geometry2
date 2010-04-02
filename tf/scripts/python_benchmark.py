import roslib
roslib.load_manifest('tf')
import rostest
import rospy
import numpy
import unittest
import sys
import time
import StringIO

import tf.transformations
import geometry_msgs.msg

from tf.msg import tfMessage

import tf

iterations = 10000

t = tf.Transformer()
def mkm():
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = "PARENT"
    m.child_frame_id = "THISFRAME"
    m.transform.translation.y = 5.0
    m.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    return m

tm = tfMessage([mkm() for i in range(20)])
s = StringIO.StringIO()
tm.serialize(s)
mstr = s.getvalue()
print len(mstr)

print tf.FasterMessage.__bases__

m2 = tf.FasterMessage()
# m2 = tfMessage()
m2.deserialize(mstr)
for m in m2.transforms:
    print type(m), sys.getrefcount(m)

started = time.time()
for i in xrange(iterations):
    m2 = tfMessage()
    # m2 = tf.FasterMessage()
    m2.deserialize(mstr)
took = time.time() - started
print "deserialize only ", iterations, "took", took, "%f us each" % (1e6 * took / iterations)

started = time.time()
for i in xrange(iterations):
    for m in tm.transforms:
        t.setTransform(m)
took = time.time() - started
print "setTransform only", iterations, "took", took, "%f us each" % (1e6 * took / iterations)

started = time.time()
for i in xrange(iterations):
    m2 = tfMessage()
    m2.deserialize(mstr)
    for m in m2.transforms:
        t.setTransform(m)
took = time.time() - started
print "deserialize+setTransform ", iterations, "took", took, "%f us each" % (1e6 * took / iterations)

from tf import TransformListener
