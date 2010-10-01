#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
import roslib; roslib.load_manifest('tf2_tools')
import rospy

import wxversion
import sys
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)

import wx
import wx.richtext
import xdot
import threading
import yaml
from tf2_msgs.srv import FrameGraph, FrameGraphResponse
import tf2_ros
import os

import rosgraph.masterapi

from tf2_tools.frame_viewer_panel import FrameViewerPanel
from tf2_tools.tf_interface import TFInterface

class FrameViewerFrame(wx.Frame):
    def __init__(self, tf_interface):
        wx.Frame.__init__(self, None, -1, "Frame Viewer", size=(1024,768))

        self.viewer = FrameViewerPanel(self, tf_interface)

        # Create menu
        menubar = wx.MenuBar()
        file = wx.Menu()
        file.Append(101, '&Load', 'Load a frame_viewer snapshot')
        file.Append(102, '&Save', 'Save a frame_viewer snapshot')
        file.Append(103, '&Export PDF', 'Export To PDF')
        menubar.Append(file, '&File')
        self.SetMenuBar(menubar)
       
        wx.EVT_MENU(self, 101, self.onLoad)
        wx.EVT_MENU(self, 102, self.onSave)
        wx.EVT_MENU(self, 103, self.onPDF)

    def onLoad(self, event): self.viewer.update_file_list(wx.LoadFileSelector("TF Snapshot", ".tf"))
    def onSave(self, event): self.viewer.tf_interface.save_yaml(wx.SaveFileSelector("TF Snapshot", ".tf"))
    def onPDF(self, event): self.viewer.tf_interface.save_pdf(wx.SaveFileSelector("PDF Export", ".pdf"))

class FrameViewerApp(wx.App):
    def __init__(self):
        wx.App.__init__(self)

    def OnInit(self):
        self.tf_interface = TFInterface()
        self.frame = FrameViewerFrame(self.tf_interface)
        self.frame.Show()
        return True

def main():
    app = FrameViewerApp()
    app.MainLoop()

if __name__ == '__main__':
    rospy.init_node('frame_viewer', anonymous=False, disable_signals=True, log_level=rospy.DEBUG)
    main()
    rospy.signal_shutdown('GUI shutdown')
