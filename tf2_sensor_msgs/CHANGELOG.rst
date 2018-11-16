^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_sensor_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.20 (2018-11-16)
-------------------

0.5.19 (2018-11-06)
-------------------

0.5.18 (2018-07-10)
-------------------

0.5.17 (2018-01-01)
-------------------
* Merge pull request `#257 <https://github.com/ros/geometry2/issues/257>`_ from delftrobotics-forks/python3
  Make tf2_py python3 compatible again
* Use python3 print function.
* Contributors: Maarten de Vries, Tully Foote

0.5.16 (2017-07-14)
-------------------
* Fix do_transform_cloud for multi-channelled pointcloud2. (`#241 <https://github.com/ros/geometry2/issues/241>`_)
* store gtest return value as int (`#229 <https://github.com/ros/geometry2/issues/229>`_)
* Document the lifetime of the returned reference for getFrameId and getTimestamp
* Find eigen in a much nicer way.
* Switch tf2_sensor_msgs over to package format 2.
* Contributors: Atsushi Watanabe, Chris Lalancette, dhood

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------

0.5.13 (2016-03-04)
-------------------
* add missing Python runtime dependency
* fix wrong comment
* Adding tests to package
* Fixing do_transform_cloud for python
  The previous code was not used at all (it was a mistake in the __init_\_.py so
  the do_transform_cloud was not available to the python users).
  The python code need some little correction (e.g there is no method named
  read_cloud but it's read_points for instance, and as we are in python we can't
  use the same trick as in c++ when we got an immutable)
* Contributors: Laurent GEORGE, Vincent Rabaud

0.5.12 (2015-08-05)
-------------------

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* ODR violation fixes and more conversions
* Fix keeping original pointcloud header in transformed pointcloud
* Contributors: Paul Bovbel, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* add support for transforming sensor_msgs::PointCloud2
* Contributors: Vincent Rabaud
