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
import PyKDL
import rospy
import tf2_ros
from sensor_msgs.point_cloud2 import read_cloud

def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(PointCloud2, to_msg_msg)

def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(PointCloud2, from_msg_msg)

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, 
                                    t.transform.translation.y, 
                                    t.transform.translation.z))

# PointStamped
def do_transform_cloud(cloud, transform):
    res = cloud.copy()
    t_kdl = transform_to_kdl(transform)
    for p_in, p_out in [ read_cloud(cloud), read_cloud(res) ]:
        p = t_kdl * PyKDL.Vector(p_in.x, p_in.y, p_in.z)
        p_out.x = p[0]
        p_out.y = p[1]
        p_out.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)
