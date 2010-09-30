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
import tf2_py
import os


class FrameViewerFrame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1, "Frame Viewer", size=(1024,768))

        menubar = wx.MenuBar()
        file = wx.Menu()

        file.Append(101, '&Load', 'Load a frame_viewer snapshot')
        file.Append(102, '&Save', 'Save a frame_viewer snapshot')
        menubar.Append(file, '&File')

        self.SetMenuBar(menubar)
        wx.EVT_MENU(self, 101, self.onLoad)
        wx.EVT_MENU(self, 102, self.onSave)


        self.needs_refresh = False
        self.new_info_text = None
        self.first_dot_data = True

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
        toolbar.AddControl(wx.StaticText(toolbar,-1,"Graph Namespace:  "))
        self.namespaces = wx.ComboBox(toolbar, -1, value = 'local', choices = ['local'], size=(-1, -1), style=wx.CB_DROPDOWN)
        self.namespaces.SetEditable(False)
        self.namespaces.SetItems(['local', 'foo'])
        toolbar.AddControl(self.namespaces)
        self.Bind(wx.EVT_COMBOBOX, self.onSelect)
        #Add some stuff here later
        toolbar.Realize();

        #Create graph_view widget
        self.widget = xdot.wxxdot.WxDotWindow(graph_viewer, -1)
        self.widget.zoom_to_fit()

        gv_vbox.Add(toolbar, 0, wx.EXPAND)
        gv_vbox.Add(self.widget, 1, wx.EXPAND)

        nb = wx.Notebook(self.content_splitter, -1, style=wx.NB_TOP | wx.WANTS_CHARS)
        nb_box = wx.BoxSizer()
        nb_box.Add(nb, 1, wx.EXPAND | wx.ALL, 4)

        #Create an info pane
        borders = wx.LEFT | wx.RIGHT | wx.TOP
        border = 4
        self.info_win = wx.ScrolledWindow(nb, -1)
        self.info_sizer = wx.BoxSizer(wx.VERTICAL)
        self.info_txt = wx.TextCtrl(self.info_win,-1,style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.info_sizer.Add(self.info_txt,1,wx.EXPAND | borders, border)
        self.info_win.SetSizer(self.info_sizer)

        #Create a tf echo pane
        echo_panel = wx.Panel(nb, -1)
        echo_box = wx.BoxSizer(wx.VERTICAL)
        echo_panel.SetSizer(echo_box)
        wx.StaticText(echo_panel, -1, "Target: ", pos=(5, 15))
        self.from_frame = wx.ComboBox(echo_panel, -1, value = 'select', choices = ['select'], pos=(60, 10), size = (200, -1), style=wx.CB_DROPDOWN)
        self.from_frame.SetEditable(False)

        wx.StaticText(echo_panel, -1, "Source: ", pos=(5, 55))
        self.to_frame = wx.ComboBox(echo_panel, -1, value = 'foo', choices = ['foo'], pos=(60, 50), size = (200, -1), style=wx.CB_DROPDOWN)
        self.to_frame.SetEditable(False)

        self.echo_txt = wx.TextCtrl(echo_panel, -1, style=wx.TE_MULTILINE|wx.TE_READONLY, pos=(5, 90), size=(255,-1))
        self.echo_txt.SetValue("Foobar")

        #Add our panels to the notebook
        nb.AddPage(self.info_win, "Info")
        nb.AddPage(echo_panel, "TF Echo")



        # Set content splitter
        #self.content_splitter.SplitHorizontally(nb, viewer, 120)
        self.content_splitter.SplitVertically(nb, viewer, 300)

        vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
        self.SetSizer(vbox)
        self.Center()

        self.Bind(wx.EVT_IDLE, self.OnIdle)

    def onSelect(self, event):
        print type(event)
        print event.GetEventObject() == self.from_frame
        print event.GetEventObject().GetValue()

    def onLoad(self, event):
        print wx.LoadFileSelector("TF Snapshot", ".tf")

    def onSave(self, event):
        print wx.SaveFileSelector("TF Snapshot", ".tf")

    def register_select_cb(self, callback):
        #Register mouse event callback for widget
        self.widget.register_select_callback(callback)

    def set_dotcode(self, dotcode):
        self.widget.set_dotcode(dotcode, None)

        #We'll only zoom to fit the first time we get data
        if self.first_dot_data:
            self.first_dot_data = False
            self.widget.zoom_to_fit()

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
        self.selected_parent = None
        self.selected_child = None
        self.data = None
        rospy.wait_for_service('~tf2_frames')
        self.srv = rospy.ServiceProxy('~tf2_frames', FrameGraph, persistent=True)

    def set_detail(self, detail):
        selected = detail.split('-')
        self.selected_parent = selected[0]
        self.selected_child = selected[1]

    def update_data(self):
        self.data = yaml.load(self.srv().frame_yaml)

    def get_dot(self):
        return self.generate_dot(self.data)

    def get_info(self):
        if self.selected_child != None:
            return self.data[self.selected_child]
        return ""

    def generate_dot(self, data):
        if len(data) == 0:
            return 'digraph G { "No tf data received" }'

        dot = 'digraph G {\n'
        for el in data: 
            map = data[el]
            node_color = 'white'
            arrow_color = 'black'
            if el == self.selected_parent:
                node_color = 'green'
            if el == self.selected_child:
                node_color = 'green'
                arrow_color = 'green'
            dot += el
            dot += '['
            dot += 'shape=ellipse style=filled fillcolor=' + node_color + ' '
            dot += 'URL="' + map['parent'] + '-' + el + '"] '
            dot += '"'+map['parent']+'" -> "'+el+'" [color=' + arrow_color + ']'
            #if map['parent'] + '-' + el == detail:
            #    dot += '[label=" '
            #    dot += 'Broadcaster: '+map['broadcaster']+'\\n'
            #    dot += 'Average rate: '+str(map['rate'])+'\\n'
            #    dot += 'Buffer length: '+str(map['buffer_length'])+'\\n' 
            #    dot += 'Most recent transform: '+str(map['most_recent_transform'])+'\\n'
            #    dot += 'Oldest transform: '+str(map['oldest_transform'])+'\\n'
            #    dot += '"];\n'
            if not map['parent'] in data:
                root = map['parent']
                if map['parent'] == self.selected_parent:
                    dot += root + '[fillcolor=green style=filled]'
        dot += 'edge [style=invis];\n'
        dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
        dot += '"Recorded at time: '+str(rospy.Time.now().to_sec())+'"[ shape=plaintext ] ;\n'
        dot += '}->"'+root+'";\n}'
        return dot

class FrameViewerApp(wx.App):
    def __init__(self):
        wx.App.__init__(self)
        self.tf_interface = TFInterface()
        #self.update_dotcode()
        self.frame.register_select_cb(self.select_cb)

    def select_cb(self, target, event):
        if event.ButtonDown(wx.MOUSE_BTN_LEFT) and target.url != None:
            self.tf_interface.set_detail(target.url)

    def update_dotcode_event(self, event):
        self.update_dotcode()

    def update_dotcode(self):
        self.tf_interface.update_data()
        dotcode = self.tf_interface.get_dot()
        self.frame.set_info_text(yaml.dump(self.tf_interface.get_info(), default_flow_style=False))
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
