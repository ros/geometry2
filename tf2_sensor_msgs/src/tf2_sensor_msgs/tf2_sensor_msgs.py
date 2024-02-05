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

from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
import PyKDL
import rospy
import tf2_ros

def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(PointCloud2, to_msg_msg)

def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(PointCloud2, from_msg_msg)

def transform_to_kdl_rotation_only(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w))

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, 
                                    t.transform.translation.y, 
                                    t.transform.translation.z))

def do_transform_cloud_with_channels(cloud, transform, channels):
    t_kdl = transform_to_kdl(transform)
    t_kdl_rot = transform_to_kdl_rotation_only(transform)
    points_out = []
    for p_in in read_points(cloud):
        fi = 0
        p_out = []
        while fi < len(cloud.fields):
            field = cloud.fields[fi]
            transformed = False
            for prefix, only_rotation in channels:
                if field.name == prefix + "x":
                    t = t_kdl if not only_rotation else t_kdl_rot
                    p = t * PyKDL.Vector(p_in[fi + 0], p_in[fi + 1], p_in[fi + 2])
                    p_out.extend(p)
                    transformed = True
                    break
            if transformed:
                fi += 3  # skip the following two fields since we've already done them
            else:
                p_out.append(p_in[fi])
                fi += 1
        points_out.append(p_out)

    res = create_cloud(transform.header, cloud.fields, points_out)
    return res

# PointStamped
def do_transform_cloud(cloud, transform):
    return do_transform_cloud_with_channels(
        cloud, transform, channels=[("", False), ("vp_", False), ("normal_", True)])
tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)
