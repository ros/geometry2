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

import roslib; roslib.load_manifest('tf2_py')
import rospy
from geometry_msgs.msg import TransformStamped


class BufferCore():

    # simple api
    def lookupTransformCore(self, target_frame, source_frame, time):
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.rotation.x = 1
        t.header.stamp = time
        t.header.frame_id = target_frame
        t.child_frame_id = source_frame
        return t

    # advanced api
    def lookupTransformFullCore(self, target_frame, target_time,
                            source_frame, source_time, fixed_frame):
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.rotation.x = 1
        t.header.stamp = time
        t.header.frame_id = target_frame
        t.child_frame_id = source_frame
        return t


    # simple api
    def canTransformCore(self, target_frame, source_frame, time):
        return True    

    # advanced api
    def canTransformFullCore(self, target_frame, target_time,
                         source_frame, source_time, fixed_frame):
        return True    

