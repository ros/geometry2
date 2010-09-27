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

import roslib; roslib.load_manifest('tf2_kdl')
import PyKDL
import rospy
import tf2_py

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, 
                                    t.transform.translation.y, 
                                    t.transform.translation.z))


# Frame
def do_transform_frame(frame, transform):
    res = transform_to_kdl(transform) * frame
    res.header = transform.header
    return res
tf2_py.TransformRegistration().add(PyKDL.Frame, do_transform_frame)


# Vector
def do_transform_vector(vector, transform):
    res = transform_to_kdl(transform) * vector
    res.header = transform.header
    return res

tf2_py.TransformRegistration().add(PyKDL.Vector, do_transform_vector)


# Twist
def do_transform_twist(twist, transform):
    res = transform_to_kdl(transform) * twist
    res.header = transform.header
    return res
tf2_py.TransformRegistration().add(PyKDL.Twist, do_transform_twist)


# Wrench
def do_transform_wrench(wrench, transform):
    res = transform_to_kdl(transform) * wrench
    res.header = transform.header
    return res
tf2_py.TransformRegistration().add(PyKDL.Wrench, do_transform_wrench)
