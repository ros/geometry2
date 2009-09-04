# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## Python utility for iterating over messages in a ROS .bag file
## See http://pr.willowgarage.com/wiki/ROS/LogFormat
# authors: jamesb, kwc

PKG = 'tf'
import roslib
roslib.load_manifest(PKG)

import sys, os

import rospy
import tf as TFX
from tf import transformations
import numpy

from tf.msg import tfMessage
import geometry_msgs.msg
from tf.srv import FrameGraph,FrameGraphResponse

def xyz_to_mat44(pos):
    return transformations.translation_matrix((pos.x, pos.y, pos.z))

def xyzw_to_mat44(ori):
    return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

## Extends tf's Transformer, adding transform methods for ROS message
## types PointStamped, QuaternionStamped and PoseStamped.
class TransformerROS(TFX.Transformer):

    ## Looks up the transform for ROS message header hdr to frame
    ## target_frame, and returns the transform as a Numpy 4x4 matrix.
    # @param target_frame The target frame
    # @param hdr          A ROS message header object

    def asMatrix(self, target_frame, hdr):
        translation,rotation = self.lookupTransform(target_frame, hdr.frame_id, hdr.stamp)
        return self.fromTranslationRotation(translation, rotation)

    ## Returns a Numpy 4x4 matrix for a transform.
    # @param translation  translation as (x,y,z)
    # @param rotation     rotation as (x,y,z,w)

    def fromTranslationRotation(self, translation, rotation):
        return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

    ## Transforms a geometry_msgs PointStamped message to frame target_frame, returns the resulting PointStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.PointStamped object

    def transformPoint(self, target_frame, ps):
        mat44 = self.asMatrix(target_frame, ps.header)
        xyz = tuple(numpy.dot(mat44, numpy.array([ps.point.x, ps.point.y, ps.point.z, 1.0])))[:3]
        r = geometry_msgs.msg.PointStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.point = geometry_msgs.msg.Point(*xyz)
        return r

    ## Transforms a geometry_msgs QuaternionStamped message to frame target_frame, returns the resulting QuaternionStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.QuaternionStamped object

    def transformQuaternion(self, target_frame, ps):
        # mat44 is frame-to-frame transform as a 4x4
        mat44 = self.asMatrix(target_frame, ps.header)

        # pose44 is the given quat as a 4x4
        pose44 = xyzw_to_mat44(ps.quaternion)

        # txpose is the new pose in target_frame as a 4x4
        txpose = numpy.dot(mat44, pose44)

        # quat is orientation of txpose
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        # assemble return value QuaternionStamped
        r = geometry_msgs.msg.QuaternionStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.quaternion = geometry_msgs.msg.Quaternion(*quat)
        return r

    ## Transforms a geometry_msgs PoseStamped message to frame target_frame, returns the resulting PoseStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.PoseStamped object

    def transformPose(self, target_frame, ps):
        # mat44 is frame-to-frame transform as a 4x4
        mat44 = self.asMatrix(target_frame, ps.header)

        # pose44 is the given pose as a 4x4
        pose44 = numpy.dot(xyz_to_mat44(ps.pose.position), xyzw_to_mat44(ps.pose.orientation))

        # txpose is the new pose in target_frame as a 4x4
        txpose = numpy.dot(mat44, pose44)

        # xyz and quat are txpose's position and orientation
        xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        # assemble return value PoseStamped
        r = geometry_msgs.msg.PoseStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat))
        return r

## Extends TransformerROS, subscribes to the /tf_message topic and
## updates the Transformer with the messages.

class TransformListener(TransformerROS):

    def __init__(self, *args):
        print "Transform Listener initing"
        super(TransformListener, self).__init__()
        rospy.Subscriber("/tf_message", tfMessage, self.transformlistener_callback)
        self.frame_graph_server = rospy.Service('~tf_frames', FrameGraph, self.frame_graph_service)

    def transformlistener_callback(self, data):
        for transform in data.transforms:
            self.setTransform(transform)
        #print self.allFramesAsString()

    def frame_graph_service(self, req):
        return FrameGraphResponse(self.allFramesAsDot())
