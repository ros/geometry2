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
import yaml
import subprocess

from tf2_msgs.srv import FrameGraph
import tf2_ros
import tf2

import rosgraph.masterapi

class TFInterface(object):
    def __init__(self, use_listener=True):
        self.buffer = tf2_ros.Buffer()
        
        self.selected_parent = None
        self.selected_child = None
        self.data = None
        self.data_timestamp = None
        self.namespace = 'local'

        if use_listener:
            self.listener = tf2_ros.TransformListener(self.buffer)
            rospy.wait_for_service('~tf2_frames')
            self.srv = rospy.ServiceProxy('~tf2_frames', FrameGraph, persistent=True)
            self.master = rosgraph.masterapi.Master(rospy.get_name())
        else:
            self.listener = None
            self.srv = None
            self.master = None
    
    def register_srv(self, namespace):
        if namespace == self.namespace:
            return
        
        if namespace == 'local':
            rospy.wait_for_service('~tf2_frames')
            self.srv = rospy.ServiceProxy('~tf2_frames', FrameGraph, persistent=True)
        else:
            rospy.wait_for_service(namespace + '/tf2_frames')
            self.srv = rospy.ServiceProxy(namespace + '/tf2_frames', FrameGraph, persistent=True)
            self.namespace = namespace

    def find_tf_namespaces(self):
        ns = []
        
        if self.master:
            services = self.master.getSystemState()[2]
            for name, providers in services:
                #we don't want to list our own node
                if name.find(rospy.get_name() + '/') == 0:
                    continue
                index = name.rfind('tf2_frames')
                if index > 0:
                    ns.append(name[:index-1])
            ns.sort()
        
        return ns

    def clear_detail(self):
        self.selected_parent = None
        self.selected_child = None

    def set_detail(self, detail):
        selected = detail.split('-')
        self.selected_parent = selected[0]
        self.selected_child = selected[1]

    def set_data_timestamp(self, timestamp):
        self.data_timestamp = timestamp

    def update_data(self, namespace, from_file=False):
        if not from_file:
            if namespace == 'local':
                frame_yaml = self.buffer.all_frames_as_yaml()
            else:
                self.register_srv(namespace)
                frame_yaml = self.srv().frame_yaml

            if self.listener is not None:
                self.data_timestamp = rospy.Time.now()

            self.data = yaml.load(frame_yaml)
        else:
            self.data = self._load_yaml_file(namespace)

    def _load_yaml_file(self, filename):
        if not filename:
            return

        with open(filename, 'r') as f:
            data = yaml.load(f)
        return data

    def save_yaml(self, full_name):
        if not full_name:
            return

        filename = full_name.rstrip('.tf')
        with open(filename+'.tf', 'w') as f:
            f.write(yaml.dump(self.data))

    def save_pdf(self, full_name):
        if not full_name:
            return

        filename = full_name.rstrip('.pdf')
        with open(filename+'.gv', 'w') as f:
            f.write(self._generate_dot(self.data, full_info=True))
        subprocess.Popen(('dot -Tpdf ' + filename + '.gv -o ' + filename + '.pdf').split(' ')).communicate()
        subprocess.Popen(('rm -rf ' + filename + '.gv').split(' ')).communicate()

    def get_dot(self):
        return self._generate_dot(self.data)

    def get_info(self):
        if self.selected_child:
            return self.data[self.selected_child]
        return ""

    def get_echo_string(self, target, source):
        try:
            msg = self.buffer.lookup_transform(target, source, rospy.Time())
            echo = 'Time: ' + str(msg.header.stamp) + '\n'
            echo += 'Target: ' + msg.header.frame_id + '\n'
            echo += 'Source: ' + msg.child_frame_id + '\n'
            echo += 'Translation: \n' + str(msg.transform.translation) + '\n'
            echo += 'Rotation Quaternion: \n' + str(msg.transform.rotation) + '\n'
            return echo
        except tf2.TransformException as e:
            return str(e)

    def get_frame_list(self):
        if not self.data:
            return []

        l = self.data.keys()

        #we also need to find the root of the tree
        for el in self.data:
            map = self.data[el]
            if not map['parent'] in self.data:
                l.append(map['parent'])

        l.sort()
        return l

    def _generate_dot(self, data, full_info=False):
        if not data:
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
            if full_info:
                dot += '[label=" '
                dot += 'Broadcaster: '+map['broadcaster']+'\\n'
                dot += 'Average rate: '+str(map['rate'])+'\\n'
                dot += 'Buffer length: '+str(map['buffer_length'])+'\\n' 
                dot += 'Most recent transform: '+str(map['most_recent_transform'])+'\\n'
                dot += 'Oldest transform: '+str(map['oldest_transform'])+'\\n'
                dot += '"];\n'
            if not map['parent'] in data:
                root = map['parent']
                if map['parent'] == self.selected_parent:
                    dot += root + '[fillcolor=green style=filled]'
        dot += 'edge [style=invis];\n'
        dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
        dot += '"Recorded at time: '+str(self.data_timestamp)+'"[ shape=plaintext ] ;\n'
        dot += '}->"'+root+'";\n}'

        return dot
