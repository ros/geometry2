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

# author: Wim Meeussen

from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped
import numpy
from transforms3d import quaternions
import rospy
import tf2_ros

import PyKDL

def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(Vector3Stamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PoseStamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PointStamped, to_msg_msg)

def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(Vector3Stamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PoseStamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PointStamped, from_msg_msg)

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))

def quaterion_from_transform(transform):
   return [transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]

def translation_from_transform(transform):
   return numpy.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])

# PointStamped
def do_transform_point(point, transform):
    point = numpy.array([point.point.x, point.point.y, point.point.z])
    p = numpy.add(quaternions.rotate_vector(point, quaterion_from_transform(transform)), translation_from_transform(transform))
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


# Vector3Stamped
def do_transform_vector3(vector3, transform):
    vector = numpy.array([vector3.vector.x, vector3.vector.y, vector3.vector.z])
    p = quaternions.rotate_vector(vector, quaterion_from_transform(transform))
    res = Vector3Stamped()
    res.vector.x = p[0]
    res.vector.y = p[1]
    res.vector.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(Vector3Stamped, do_transform_vector3)

# PoseStamped
def do_transform_pose(pose, transform):
    position = numpy.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
    p = numpy.add(quaternions.rotate_vector(position, quaterion_from_transform(transform)), translation_from_transform(transform));
    rotation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w];
    q = quaternions.qmult(rotation, quaterion_from_transform(transform));
    res = PoseStamped()
    res.pose.position.x = p[0]
    res.pose.position.y = p[1]
    res.pose.position.z = p[2]
    res.pose.orientation.w = q[0]
    res.pose.orientation.x = q[1]
    res.pose.orientation.y = q[2]
    res.pose.orientation.z = q[3]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PoseStamped, do_transform_pose)
