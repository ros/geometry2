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

import os
import sys
import yaml

import wxversion
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)

import wx
import wx.richtext
import xdot

class FrameViewerPanel(wx.Panel):
    def __init__(self, parent, tf_interface):
        wx.Panel.__init__(self, parent, -1)

        self.tf_interface   = tf_interface
        self.namespace      = ('local', False)
        self.tf_namespaces = []
        self.loaded_files = []
        self.need_dot_zoom = True

        #Create a main pane
        vbox = wx.BoxSizer(wx.VERTICAL)
        
        #Create a splitter
        self.content_splitter = wx.SplitterWindow(self, -1, style=wx.SP_LIVE_UPDATE)
        self.content_splitter.SetMinimumPaneSize(24)
        self.content_splitter.SetSashGravity(0.85)

        #Create a viewer panel
        viewer = wx.Panel(self.content_splitter, -1)

        #Create a graph panel
        graph_viewer = wx.Panel(viewer, -1)
        gv_vbox = wx.BoxSizer(wx.VERTICAL)
        graph_viewer.SetSizer(gv_vbox)

        viewer_box = wx.BoxSizer()
        viewer_box.Add(graph_viewer,1,wx.EXPAND | wx.ALL, 4)
        viewer.SetSizer(viewer_box)

        #Construct toolbar
        toolbar = wx.ToolBar(graph_viewer, -1)
        toolbar.AddControl(wx.StaticText(toolbar, -1, "Graph Namespace:  "))
        self.namespaces = wx.ComboBox(toolbar, 1, value='local', choices=['local'], size=(-1, -1), style=wx.CB_DROPDOWN)
        self.namespaces.Editable = False
        toolbar.AddControl(self.namespaces)
        self.Bind(wx.EVT_COMBOBOX, self.on_select_ns, id=1)

        toolbar.AddControl(wx.Button(toolbar, 1, 'Refresh List'))
        self.Bind(wx.EVT_BUTTON, self.on_refresh_list, id=1)
        toolbar.Realize();

        #Create graph_view widget
        self.widget = xdot.wxxdot.WxDotWindow(graph_viewer, -1)
        self.widget.zoom_to_fit()

        gv_vbox.Add(toolbar,     0, wx.EXPAND)
        gv_vbox.Add(self.widget, 1, wx.EXPAND)

        nb = wx.Notebook(self.content_splitter, -1, style=wx.NB_TOP | wx.WANTS_CHARS)
        nb_box = wx.BoxSizer()
        nb_box.Add(nb, 1, wx.EXPAND | wx.ALL, 4)

        #Create an info pane
        borders = wx.LEFT | wx.RIGHT | wx.TOP
        border = 4
        self.info_win = wx.ScrolledWindow(nb, -1)
        self.info_sizer = wx.BoxSizer(wx.VERTICAL)
        self.info_txt = wx.TextCtrl(self.info_win, -1, style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.info_sizer.Add(self.info_txt, 1, wx.EXPAND | borders, border)
        self.info_win.SetSizer(self.info_sizer)

        #Create a tf echo pane
        echo_panel = wx.Panel(nb, -1)
        echo_box = wx.BoxSizer(wx.VERTICAL)
        echo_panel.SetSizer(echo_box)
        wx.StaticText(echo_panel, -1, "Target: ", pos=(5, 15))
        self.from_frame = wx.ComboBox(echo_panel, 2, value='Select Target', choices=['Select Target'], pos=(60, 10), size=(200, -1), style=wx.CB_DROPDOWN)
        self.from_frame.SetEditable(False)
        self.Bind(wx.EVT_COMBOBOX, self.on_select_target, id=2)

        wx.StaticText(echo_panel, -1, "Source: ", pos=(5, 55))
        self.to_frame = wx.ComboBox(echo_panel, 3, value='Select Source', choices=['Select Source'], pos=(60, 50), size=(200, -1), style=wx.CB_DROPDOWN)
        self.to_frame.SetEditable(False)
        self.Bind(wx.EVT_COMBOBOX, self.on_select_source, id=3)

        self.echo_txt = wx.TextCtrl(echo_panel, -1, style=wx.TE_MULTILINE | wx.TE_READONLY, pos=(5, 90), size=(255,-1))
        self.echo_txt.SetValue("Foobar")

        #Add our panels to the notebook
        nb.AddPage(self.info_win, "Info")
        nb.AddPage(echo_panel, "TF Echo")

        # Set content splitter
        #self.content_splitter.SplitHorizontally(nb, viewer, 120)

        vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
        self.SetSizer(vbox)
        self.Center()

        self.content_splitter.SplitVertically(nb, viewer, 300)

        self.timer = wx.Timer(self)

        self.Bind(wx.EVT_TIMER, lambda event: self.update_tf_data(), self.timer)
        self.widget.register_select_callback(self.select_cb)

        self.timer.Start(1000)

    def update_file_list(self, file):
        if file:
            filename = file.rstrip('.tf')+'.tf'
            self.loaded_files.append(filename)
            self.refresh_list()
            self.namespaces.SetValue(filename)
            cmd = wx.CommandEvent(wx.EVT_COMBOBOX.evtType[0])
            cmd.SetEventObject(self.namespaces)
            cmd.SetId(self.namespaces.GetId())
            self.namespaces.GetEventHandler().ProcessEvent(cmd)

    def update_tf_data(self):
        self.tf_interface.update_data(*self.namespace)

        dotcode    = self.tf_interface.get_dot()
        frame_list = self.tf_interface.get_frame_list()

        self.to_frame.SetItems(frame_list)
        self.from_frame.SetItems(frame_list)
        self.set_info_text(yaml.dump(self.tf_interface.get_info(), default_flow_style=False))
        self.set_dotcode(dotcode)

    def set_dotcode(self, dotcode):
        if not dotcode:
            return
        
        self.widget.set_dotcode(dotcode, None)

        #We'll only zoom to fit the first time we get data
        if self.need_dot_zoom:
            self.need_dot_zoom = False
            self.widget.zoom_to_fit()

        wx.CallAfter(self.Refresh)

    def set_info_text(self, text):
        self.info_txt.Value = text
        self.Refresh()

    ## Event handling

    def select_cb(self, target, event):
        if event.ButtonDown(wx.MOUSE_BTN_LEFT) and target.url is not None:
            self.tf_interface.set_detail(target.url)

    def on_refresh_list(self, event):
        self.refresh_list()

    def refresh_list(self):
        self.tf_namespaces = self.tf_interface.find_tf_namespaces()
        self.namespaces.Items = ['local'] + self.tf_namespaces + self.loaded_files
        
    def on_select_ns(self, event):
        value = event.EventObject.Value
        #we need to check if we're looking up a namespace or just reading a file
        if value == self.namespace[0]:
            return

        if value == 'local' or self.tf_namespaces.count(value) > 0:
            self.namespace = (value, False)
        else:
            self.namespace = (value, True)

        self.need_dot_zoom = True
        
    def on_select_target(self, event):
        print "target"
        print type(event)
        print event.EventObject.Value

    def on_select_source(self, event):
        print "source"
        print type(event)
        print event.EventObject.Value
