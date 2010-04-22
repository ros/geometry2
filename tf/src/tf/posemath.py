PKG = 'tf'
import roslib
roslib.load_manifest(PKG)

from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import tf
import rospy
import numpy

class PoseMath(object):
    
    # Initializers

    def __init__(self, msg):
        self.msg = msg

    @staticmethod
    def fromMatrix(m):
        (x, y, z) = (m[0, 3], m[1, 3], m[2, 3])
        q = transformations.quaternion_from_matrix(m)
        return PoseMath(Pose(Point(x, y, z), Quaternion(*q)))

    @staticmethod
    def fromTf(tf):
        position, quaternion = tf
        return PoseMath(Pose(Point(*position), Quaternion(*quaternion)))

    @staticmethod
    def fromEuler(x, y, z, Rx, Ry, Rz):
        q = transformations.quaternion_from_euler(Rx, Ry, Rz)
        return PoseMath(Pose(Point(x, y, z), Quaternion(*q)))

    @staticmethod
    def fromCamParams(self, cv, rvec, tvec):
        """ For use with FindExtrinsicCameraParams2 """
        m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
                          [ 0, 0, 0, tvec[1,0] ], 
                          [ 0, 0, 0, tvec[2,0] ], 
                          [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
        cv.Rodrigues2(rvec, m[:2,:2])
        return self.fromMatrix(m)

    # Operators

    def __mul__(self, other):
        m = numpy.dot(self.asMatrix(), other.asMatrix())
        return PoseMath.fromMatrix(m)

    def __invert__(self):
        inv = numpy.linalg.inv(self.asMatrix())
        return PoseMath.fromMatrix(inv)

    # Representations

    def __repr__(self):
        return repr(self.msg)

    def asMessage(self):
        return self.msg

    def asMatrix(self):
        translation = (self.msg.position.x, self.msg.position.y, self.msg.position.z)
        rotation = (self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w)
        return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))        

    def asEuler(self):
        tmp = transformations.euler_from_quaternion((self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w))
        return (self.msg.position.x, self.msg.position.y, self.msg.position.z, tmp[0], tmp[1], tmp[2])

    def asTf(self):
        p = self.msg.position
        q = self.msg.orientation
        return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
