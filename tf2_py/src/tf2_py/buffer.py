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


class Buffer():
    def __init__(self):
        self.registration = TransformRegistration()
        

    # transform simple api
    def transform(self, object_stamped, target_frame, timeout=rospy.Duration(0.0)):
        do_transform = self.registration.get(type(object_stamped))
        if not do_transform:
            raise TypeException('Type %s if not loaded or supported'%type(object_stamped))
        return do_transform(object_stamped, lookupTransform(target_frame, object_stamped.header.frame_id,
                                                            object_stamped.header.stamp, timeout))
    
    # simple api
    def lookupTransform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        canTransform(target_frame, source_frame, time, timeout)
        return buffer_core.lookupTransform(target_frame, source_frame, time)


    # simple api
    def canTransform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        start_time = rospy.Time.now()
        while (rospy.Time.now() < start_time + timeout and 
               not buffer_core.canTransform(target_frame, source_frame, time)):
            rospy.Duration(0.05).sleep()
        return buffer_core.canTransform(target_frame, source_frame, time)
    


class TypeException(Exception):
    def __init__(self, errstr):
        self.errstr = errstr



class TransformRegistration():
    __type_map = {}
    
    def add(self, key, callback):
        __type_map[key] = callback

    def get(self, key):
        if not key in __type_map:
            return None
        else:
            return __type_map[key]
