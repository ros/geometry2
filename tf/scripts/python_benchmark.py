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

def deserel_to_string(o):
    s = StringIO.StringIO()
    o.serialize(s)
    return s.getvalue()

mstr = deserel_to_string(tm)

class Timer:
    def __init__(self, func):
        self.func = func
    def mean(self, iterations = 1000000):
        started = time.time()
        for i in xrange(iterations):
            self.func()
        took = time.time() - started
        return took / iterations
        
import tf.msg
import tf.cMsg
for t in [tf.msg.tfMessage, tf.cMsg.tfMessage]:
    m2 = t()
    m2.deserialize(mstr)
    for m in m2.transforms:
        print type(m), sys.getrefcount(m)
    assert deserel_to_string(m2) == mstr, "deserel screwed up for type %s" % repr(t)

    m2 = t()
    print "deserialize only ", 1e6 * Timer(lambda: m2.deserialize(mstr)).mean(), "us each"

sys.exit(0)

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
