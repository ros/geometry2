#!/usr/bin/python

PKG = 'kdl'
import roslib; roslib.load_manifest(PKG)

import subprocess
import sys
import unittest
import os

class TestKDL(unittest.TestCase):
    def test_executables(self):
        pkg = roslib.packages.get_pkg_dir('kdl')
        folder = '/build/kdl-tar/build/tests/'
        tests = ['framestest', 'inertiatest', 'jacobiantest', 'kinfamtest', 'solvertest']
        for t in tests:
            print '>>>>>>>>>>>>> Running %s'%(pkg+folder+t)
            self.assertEquals(0, subprocess.call([pkg+folder+t]))
        

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_kdl', TestKDL)


