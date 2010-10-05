# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Tim Field

PKG = 'tf2_tools'
import roslib; roslib.load_manifest(PKG)

import wx

from rxbag import bag_helper, TopicMessageView

def get_rxbag_plugins():
    return [(FrameViewerView, None, ['tf/tfMessage'])]

from tf2_tools.frame_viewer_panel import FrameViewerPanel
from tf2_tools.tf_interface       import TFInterface

class FrameViewerView(TopicMessageView):
    name = 'Frame Viewer'

    def __init__(self, timeline, parent):
        TopicMessageView.__init__(self, timeline, parent)

        self.tf_interface  = TFInterface(use_listener=False)
        self.viewer        = FrameViewerPanel(self.parent, self.tf_interface, False)
        self.buffer_length = roslib.rostime.Duration(10.0)

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)

        topic, _, t = msg_details[:3]

        tf_entries = list(self.timeline.get_entries_with_bags(topic, t - self.buffer_length, t))
        self.tf_interface.buffer.clear()
        for bag, entry in tf_entries:
            _, msg, msg_stamp = self.timeline.read_message(bag, entry.position)
            for transform in msg.transforms:
                self.tf_interface.buffer.set_transform(transform, '')
        self.tf_interface.set_data_timestamp(t.to_sec())

    def message_cleared(self):
        TopicMessageView.message_cleared(self)

        self.tf_interface.buffer.clear()
