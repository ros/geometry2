^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2014-05-07)
------------------

0.5.3 (2014-02-21)
------------------

0.5.2 (2014-02-20)
------------------

0.5.1 (2014-02-14)
------------------

0.5.0 (2014-02-14)
------------------

0.4.10 (2013-12-26)
-------------------

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* updating install rule for view_frames.py fixes `#44 <https://github.com/ros/geometry_experimental/issues/44>`_

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_

0.3.6 (2013-03-03)
------------------

0.3.5 (2013-02-15 14:46)
------------------------
* 0.3.4 -> 0.3.5

0.3.4 (2013-02-15 13:14)
------------------------
* 0.3.3 -> 0.3.4

0.3.3 (2013-02-15 11:30)
------------------------
* 0.3.2 -> 0.3.3

0.3.2 (2013-02-15 00:42)
------------------------
* 0.3.1 -> 0.3.2

0.3.1 (2013-02-14)
------------------
* 0.3.0 -> 0.3.1

0.3.0 (2013-02-13)
------------------
* switching to version 0.3.0
* removing packages with missing deps
* catkinizing geometry-experimental
* catkinizing tf2_tools
* strip out rx dependencies
* Some fixes to make things work with rxbag
* Threading ns list
* merge tf2_cpp and tf2_py into tf2_ros
* Now catching exceptions correctly with echo
* Working version of tf echo
* Making sure to clear details when switching frames
* Changing file format to tf
* First cut at loading, saving, and exporting support
* tf frame viewer is now an rxbag plugin
* Can now connect to any node in the system that has a tf2 buffer
* Now populates namespaces as well
* Now populates a frame list on the fly
* Got the GUI set up for a bunch of features, now just have to implement the backend of them
* Persistent service call to speed things up. Also, coloring on click
* Adding a first version of frame_viewer
* Adding xdot as a dep in prep for frame_viewer
* working view frames
* call new service
* new version of view_frames in new tf2_tools package
