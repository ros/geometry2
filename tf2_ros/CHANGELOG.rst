^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.10 (2013-12-26)
-------------------
* adding support for static transforms in python listener. Fixes `#46 <https://github.com/ros/geometry_experimental/issues/46>`_
* Contributors: Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* fixing pytf failing to sleep https://github.com/ros/geometry/issues/30
* moving python documentation to tf2_ros from tf2 to follow the code
* Fixed static_transform_publisher duplicate check, added rostest.

0.4.7 (2013-08-28)
------------------
* fixing new conditional to cover the case that time has not progressed yet port forward of `ros/geometry#35 <https://github.com/ros/geometry/issues/35>`_ in the python implementation
* fixing new conditional to cover the case that time has not progressed yet port forward of `ros/geometry#35 <https://github.com/ros/geometry/issues/35>`_

0.4.6 (2013-08-28)
------------------
* patching python implementation for `#24 <https://github.com/ros/geometry_experimental/issues/24>`_ as well
* Stop waiting if time jumps backwards.  fixes `#24 <https://github.com/ros/geometry_experimental/issues/24>`_
* patch to work around uninitiaized time. `#30 https://github.com/ros/geometry/issues/30`_
* Removing unnecessary CATKIN_DEPENDS  `#18 <https://github.com/ros/geometry_experimental/issues/18>`_

0.4.5 (2013-07-11)
------------------
* Revert "cherrypicking groovy patch for `#10 <https://github.com/ros/geometry_experimental/issues/10>`_ into hydro"
  This reverts commit 296d4916706d64f719b8c1592ab60d3686f994e1.
  It was not starting up correctly.
* fixing usage string to show quaternions and using quaternions in the test app
* cherrypicking groovy patch for `#10 <https://github.com/ros/geometry_experimental/issues/10>`_ into hydro

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.
* reviving unrun unittest and adding CATKIN_ENABLE_TESTING guards

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------
* adding queue accessors lost in the new API
* exposing dedicated thread logic in BufferCore and checking in Buffer
* adding methods to enable backwards compatability for passing through to tf::Transformer

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* fixing return by value for tranform method without preallocatoin
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_
* Added link against catkin_LIBRARIES for tf2_ros lib, also CMakeLists.txt clean up

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
* Merge pull request `#2 <https://github.com/ros/geometry_experimental/issues/2>`_ from KaijenHsiao/groovy-devel
  added setup.py and catkin_python_setup() to tf2_ros
* added setup.py and catkin_python_setup() to tf2_ros
* fixing cmake target collisions
* fixing catkin message dependencies
* removing packages with missing deps
* catkin fixes
* catkinizing geometry-experimental
* catkinizing tf2_ros
* catching None result in buffer client before it becomes an AttributeError, raising tf2.TransformException instead
* oneiric linker fixes, bump version to 0.2.3
* fix deprecated use of Header
* merged faust's changes 864 and 865 into non_optimized branch: BufferCore instead of Buffer in TransformListener, and added a constructor that takes a NodeHandle.
* add buffer server binary
* fix compilation on 32bit
* add missing file
* build buffer server
* TransformListener only needs a BufferCore
* Add TransformListener constructor that takes a NodeHandle so you can specify a callback queue to use
* Add option to use a callback queue in the message filter
* move the message filter to tf2_ros
* add missing std_msgs dependency
* missed 2 lines in last commit
* removing auto clearing from listener for it's unexpected from a library
* static transform tested and working
* subscriptions to tf_static unshelved
* static transform publisher executable running
* latching static transform publisher
* cleaning out old commented code
* Only query rospy.Time.now() when the timeout is greater than 0
* debug comments removed
* move to tf2_ros completed. tests pass again
* merge tf2_cpp and tf2_py into tf2_ros
