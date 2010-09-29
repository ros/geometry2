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
import time
import yaml
from tf2_msgs.srv import FrameGraph, FrameGraphResponse
import tf2_py

def generate_dot(data):
    if len(data) == 0:
        return 'digraph G { "No tf data received" }'

    dot = 'digraph G {\n'
    for el in data: 
        map = data[el]
        dot += '"'+map['parent']+'" -> "'+el+'"'
        dot += '[label=" '
        dot += 'Broadcaster: '+map['broadcaster']+'\\n'
        dot += 'Average rate: '+str(map['rate'])+'\\n'
        dot += 'Buffer length: '+str(map['buffer_length'])+'\\n' 
        dot += 'Most recent transform: '+str(map['most_recent_transform'])+'\\n'
        dot += 'Oldest transform: '+str(map['oldest_transform'])+'\\n'
        dot += '"];\n'
        if not map['parent'] in data:
            root = map['parent']
    dot += 'edge [style=invis];\n'
    dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
    dot += '"Recorded at time: '+str(rospy.Time.now().to_sec())+'"[ shape=plaintext ] ;\n'
    dot += '}->"'+root+'";\n}'
    return dot

class RepeatTimer(threading.Thread):
    def __init__(self, timeout, fn):
        threading.Thread.__init__(self)
        self.daemon = True
        self.timeout = timeout
        self.fn = fn
        self.keep_going = True

    def run(self):
        time.sleep(self.timeout)
        while self.keep_going:
            self.fn()
            time.sleep(self.timeout)

    def stop(self):
        self.keep_going = False
        
class FrameViewerFrame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1, "Action Viewer", size=(1024,768))

        self.needs_refresh = False
        self.new_info_text = None

        #Create a main pane
        vbox = wx.BoxSizer(wx.VERTICAL)
        
        #Create a splitter
        self.content_splitter = wx.SplitterWindow(self, -1,style = wx.SP_LIVE_UPDATE)
        self.content_splitter.SetMinimumPaneSize(24)
        self.content_splitter.SetSashGravity(0.85)

        #Create a viewer panel
        viewer = wx.Panel(self.content_splitter,-1)

        #Create a graph panel
        graph_viewer = wx.Panel(viewer, -1)
        gv_vbox = wx.BoxSizer(wx.VERTICAL)
        graph_viewer.SetSizer(gv_vbox)

        viewer_box = wx.BoxSizer()
        viewer_box.Add(graph_viewer,1,wx.EXPAND | wx.ALL, 4)
        viewer.SetSizer(viewer_box)

        #Construct toolbar
        toolbar = wx.ToolBar(graph_viewer, -1)
        toolbar.AddControl(wx.StaticText(toolbar,-1,"Foo:"))
        #Add some stuff here later
        toolbar.Realize();

        #Create graph_view widget
        self.widget = xdot.wxxdot.WxDotWindow(graph_viewer, -1)
        self.widget.zoom_to_fit()

        gv_vbox.Add(toolbar, 0, wx.EXPAND)
        gv_vbox.Add(self.widget, 1, wx.EXPAND)

        #Create an info pane
        borders = wx.LEFT | wx.RIGHT | wx.TOP
        border = 4
        self.info_win = wx.ScrolledWindow(self.content_splitter, -1)
        self.info_sizer = wx.BoxSizer(wx.VERTICAL)
        self.info_sizer.Add(wx.StaticText(self.info_win,-1,"Info:"),0, borders, border)
        self.info_txt = wx.TextCtrl(self.info_win,-1,style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.info_sizer.Add(self.info_txt,1,wx.EXPAND | borders, border)
        self.info_win.SetSizer(self.info_sizer)

        # Set content splitter
        self.content_splitter.SplitHorizontally(self.info_win, viewer, 120)

        vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
        self.SetSizer(vbox)
        self.Center()

        self.Bind(wx.EVT_IDLE, self.OnIdle)


    def register_select_cb(self, callback):
        #Register mouse event callback for widget
        self.widget.register_select_callback(callback)

    def set_dotcode(self, dotcode):
        self.widget.set_dotcode(dotcode, None)
        self.needs_refresh = True
        wx.PostEvent(self.GetEventHandler(), wx.IdleEvent())

    def set_info_text(self, text):
        self.new_info_text = text

    def OnIdle(self, event):
        if self.new_info_text is not None:
            self.info_txt.SetValue(self.new_info_text)
            self.new_info_text = None
        if self.needs_refresh:
            self.Refresh()
            self.needs_refresh = False

class TFInterface:
    def __init__(self):
        self.buffer = tf2_py.Buffer()
        self.listener = tf2_py.TransformListener(self.buffer)

    def get_dot(self):
        print "wait"
        rospy.wait_for_service('~tf2_frames')
        print "got"
        srv = rospy.ServiceProxy('~tf2_frames', FrameGraph)
        data = yaml.load(srv().frame_yaml)
        return generate_dot(data)

class FrameViewerApp(wx.App):
    def __init__(self):
        wx.App.__init__(self)
        self.tf_interface = TFInterface()
        #self.update_dotcode()
        self.frame.register_select_cb(self.select_cb)

    def select_cb(self, target, event):
        print "here"
        return
        if event.ButtonUp(wx.MOUSE_BTN_LEFT):
            self.frame.set_info_text(target.url)

            if target.url is not None:
                key = '/%s' % target.url
                if self.watchers.has_key(key):
                    wx.CallAfter(self.show_window, self.watchers[key])

    def update_dotcode_event(self, event):
        self.update_dotcode()

    def update_dotcode(self):
        dotcode = self.tf_interface.get_dot()
        wx.CallAfter(self.frame.set_dotcode, dotcode)

    def OnInit(self):
        self.frame = FrameViewerFrame()
        self.timer = wx.Timer(self.frame)
        self.frame.Bind(wx.EVT_TIMER, self.update_dotcode_event, self.timer)
        self.timer.Start(1000)

        self.frame.Show()
        return True

def main():
    app = FrameViewerApp()
    app.MainLoop()

def foo(req):
    return FrameGraphResponse()

if __name__ == '__main__':
    rospy.init_node('frame_viewer', anonymous=False, disable_signals=True, log_level=rospy.DEBUG)
    server = rospy.Service('~foobar', FrameGraph, foo)
    main()
    rospy.signal_shutdown('GUI shutdown')
