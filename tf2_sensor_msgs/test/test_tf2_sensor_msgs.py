#!/usr/bin/env python

from __future__ import print_function

import unittest
import struct
import tf2_sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from tf2_ros import TransformStamped
import copy

## A sample python unit test
class PointCloudConversions(unittest.TestCase):
    def setUp(self):
        self.point_cloud_in = point_cloud2.PointCloud2()
        self.point_cloud_in.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1)]

        self.point_cloud_in.point_step = 4 * 3
        self.point_cloud_in.height = 1
        # we add two points (with x, y, z to the cloud)
        self.point_cloud_in.width = 2
        self.point_cloud_in.row_step = self.point_cloud_in.point_step * self.point_cloud_in.width

        points = [1, 2, 0, 10, 20, 30]
        self.point_cloud_in.data = struct.pack('%sf' % len(points), *points)


        self.transform_translate_xyz_300 = TransformStamped()
        self.transform_translate_xyz_300.transform.translation.x = 300
        self.transform_translate_xyz_300.transform.translation.y = 300
        self.transform_translate_xyz_300.transform.translation.z = 300
        self.transform_translate_xyz_300.transform.rotation.w = 1  # no rotation so we only set w

        assert(list(point_cloud2.read_points(self.point_cloud_in)) == [(1.0, 2.0, 0.0), (10.0, 20.0, 30.0)])

    def test_simple_transform(self):
        old_data = copy.deepcopy(self.point_cloud_in.data)  # deepcopy is not required here because we have a str for now
        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(self.point_cloud_in, self.transform_translate_xyz_300)

        k = 300
        expected_coordinates = [(1+k, 2+k, 0+k), (10+k, 20+k, 30+k)]
        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        print("new_points are %s" % new_points)
        assert(expected_coordinates == new_points)
        assert(old_data == self.point_cloud_in.data)  # checking no modification in input cloud


## A simple unit test for tf2_sensor_msgs.do_transform_cloud (multi channel version)
class PointCloudConversionsMultichannel(unittest.TestCase):
    TRANSFORM_OFFSET_DISTANCE = 300

    def setUp(self):
        self.point_cloud_in = point_cloud2.PointCloud2()
        self.point_cloud_in.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1),
                                PointField('index', 12, PointField.INT32, 1)]

        self.point_cloud_in.point_step = 4 * 4
        self.point_cloud_in.height = 1
        # we add two points (with x, y, z to the cloud)
        self.point_cloud_in.width = 2
        self.point_cloud_in.row_step = self.point_cloud_in.point_step * self.point_cloud_in.width

        self.points = [(1.0, 2.0, 0.0, 123), (10.0, 20.0, 30.0, 456)]
        for point in self.points:
            self.point_cloud_in.data += struct.pack('3fi', *point)

        self.transform_translate_xyz_300 = TransformStamped()
        self.transform_translate_xyz_300.transform.translation.x = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.translation.y = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.translation.z = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.rotation.w = 1  # no rotation so we only set w

        assert(list(point_cloud2.read_points(self.point_cloud_in)) == self.points)

    def test_simple_transform_multichannel(self):
        old_data = copy.deepcopy(self.point_cloud_in.data)  # deepcopy is not required here because we have a str for now
        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(self.point_cloud_in, self.transform_translate_xyz_300)

        expected_coordinates = []
        for point in self.points:
           expected_coordinates += [( 
                    point[0] + self.TRANSFORM_OFFSET_DISTANCE,
                    point[1] + self.TRANSFORM_OFFSET_DISTANCE,
                    point[2] + self.TRANSFORM_OFFSET_DISTANCE,
                    point[3] # index channel must be kept same
                )]

        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        print("new_points are %s" % new_points)
        assert(expected_coordinates == new_points)
        assert(old_data == self.point_cloud_in.data)  # checking no modification in input cloud


## A simple unit test for tf2_sensor_msgs.do_transform_cloud_with_channels
class PointCloudConversionsMulti3Dchannel(unittest.TestCase):
    TRANSFORM_OFFSET_DISTANCE = 300

    def setUp(self):
        self.point_cloud_in = point_cloud2.PointCloud2()
        self.point_cloud_in.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1),
                                PointField('index', 12, PointField.INT32, 1),
                                PointField('vp_x', 16, PointField.FLOAT32, 1),
                                PointField('vp_y', 20, PointField.FLOAT32, 1),
                                PointField('vp_z', 24, PointField.FLOAT32, 1),
                                PointField('normal_x', 28, PointField.FLOAT32, 1),
                                PointField('normal_y', 32, PointField.FLOAT32, 1),
                                PointField('normal_z', 36, PointField.FLOAT32, 1)]

        self.point_cloud_in.point_step = len(self.point_cloud_in.fields) * 4
        self.point_cloud_in.height = 1
        # we add two points (with x, y, z to the cloud)
        self.point_cloud_in.width = 2
        self.point_cloud_in.row_step = self.point_cloud_in.point_step * self.point_cloud_in.width

        self.points = [(1.0, 2.0, 0.0, 123, 1.0, 2.0, 0.0, 1.0, 2.0, 0.0),
                       (10.0, 20.0, 30.0, 456, 10.0, 20.0, 30.0, 10.0, 20.0, 30.0)]
        for point in self.points:
            self.point_cloud_in.data += struct.pack('3fi6f', *point)

        self.transform_translate_xyz_300 = TransformStamped()
        self.transform_translate_xyz_300.transform.translation.x = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.translation.y = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.translation.z = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300.transform.rotation.w = 1  # no rotation so we only set w

        self.transform_translate_xyz_300_flip_x = TransformStamped()
        self.transform_translate_xyz_300_flip_x.transform.translation.x = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300_flip_x.transform.translation.y = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300_flip_x.transform.translation.z = self.TRANSFORM_OFFSET_DISTANCE
        self.transform_translate_xyz_300_flip_x.transform.rotation.x = 1

        assert(list(point_cloud2.read_points(self.point_cloud_in)) == self.points)

    def test_simple_transform_multichannel(self):
        old_data = copy.deepcopy(self.point_cloud_in.data)  # deepcopy is not required here because we have a str for now
        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(self.point_cloud_in, self.transform_translate_xyz_300)

        # only translation

        expected_coordinates = [(301.0, 302.0, 300.0, 123, 301.0, 302.0, 300.0, 1.0, 2.0, 0.0),
                                (310.0, 320.0, 330.0, 456, 310.0, 320.0, 330.0, 10.0, 20.0, 30.0)]

        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        print("new_points are %s" % new_points)
        assert(expected_coordinates == new_points)
        assert(old_data == self.point_cloud_in.data)  # checking no modification in input cloud

        # both translation and rotation

        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(self.point_cloud_in, self.transform_translate_xyz_300_flip_x)

        expected_coordinates = [(301.0, 298.0, 300.0, 123, 301.0, 298.0, 300.0, 1.0, -2.0, 0.0),
                                (310.0, 280.0, 270.0, 456, 310.0, 280.0, 270.0, 10.0, -20.0, -30.0)]

        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        print("new_points are %s" % new_points)
        assert(expected_coordinates == new_points)
        assert(old_data == self.point_cloud_in.data)  # checking no modification in input cloud


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("test_tf2_sensor_msgs", "test_point_cloud_conversion", PointCloudConversions)
    rosunit.unitrun("test_tf2_sensor_msgs", "test_point_cloud_conversion", PointCloudConversionsMultichannel)
    rosunit.unitrun("test_tf2_sensor_msgs", "test_point_cloud_conversion", PointCloudConversionsMulti3Dchannel)

