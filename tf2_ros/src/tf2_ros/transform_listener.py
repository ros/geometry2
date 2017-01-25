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

import roslib; roslib.load_manifest('tf2_ros')
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage

class TransformListener():
    """
    :class:`TransformListener` is a convenient way to listen for coordinate frame transformation info.
    This class takes an object that instantiates the :class:`BufferInterface` interface, to which
    it propagates changes to the tf frame graph.
    """
    def __init__(self, buffer):
        """
        .. function:: __init__(buffer)

            Constructor.

            :param buffer: The buffer to propagate changes to when tf info updates.
        """
        self.buffer = buffer
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self.static_callback)
    
    def __del__(self):
        self.unregister()

    def unregister(self):
        """
        Unregisters all tf subscribers.
        """
        self.tf_sub.unregister()
        self.tf_static_sub.unregister()

    def callback(self, data):
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data):
        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)
