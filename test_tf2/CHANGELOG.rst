^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_tf2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.8 (2015-03-17)
------------------
* remove useless Makefile files
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------
* Removed AsyncSpinner workaround
* Contributors: Esteve Fernandez

0.5.4 (2014-05-07)
------------------
* Clean up warnings about autostart and add some assertions for coverage
* Contributors: Tully Foote

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
* fixing kdl linking for tests
* Contributors: Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* Fixed static_transform_publisher duplicate check, added rostest.

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------
* fixing quaternion in unit test and adding a timeout on the waitForServer
* fixing usage string to show quaternions and using quaternions in the test app
* removing redundant declaration
* disabling whole cmake invocation in test_tf2 when not CATKIN_ENABLE_TESTING

0.4.4 (2013-07-09)
------------------

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------
* fixing test target dependencies
* fixing colliding target names between geometry and geometry_experimental
* stripping tf2_ros dependency from tf2_bullet.  Test was moved to test_tf2

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* switching to console_bridge from rosconsole
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  fixing overmatch on search and replace
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_

0.3.6 (2013-03-03)
------------------

0.3.5 (2013-02-15 14:46)
------------------------

0.3.4 (2013-02-15 13:14)
------------------------

0.3.3 (2013-02-15 11:30)
------------------------

0.3.2 (2013-02-15 00:42)
------------------------

0.3.1 (2013-02-14)
------------------

0.3.0 (2013-02-13)
------------------
* removing packages with missing deps
* catkinizing geometry-experimental
* add boost linkage
* fixing test for header cleanup
* fixing usage of bullet for migration to native bullet
* Cleanup on test code, all tests pass
* cleanup on optimized tests, still failing
* Cleanup in compound transform test
* Adding more frames to compound transform case
* Compound transform test fails on optimized case after more frames added
* Compound transform test has more frames in it
* Cleanup of compount transform test
* Compound transform at root node test fails for optimized branch
* compount transform test, non-optimized
* time-varying tests with different time-steps for optimized case
* Time-varying test inserts data at different time-steps for non-optimized case
* Helix (time-varying) test works on optimized branch
* Adding more complicated case to helix test
* Adding helix test for time-varying transforms in non-optimized case
* Corrected ring45 values in buffer core test
* Corrected values of ring45 test for non-optimized case
* Ring 45 test running on non-optimized tf2 branch, from Tully's commit r880
* filling out ring test case which finds errors in the optimization
* Add option to use a callback queue in the message filter
* another out-the-back test
* move the message filter to tf2_ros
* fix warnings
* merge from tf_rework
* tf2::MessageFilter + tests.  Still need to change it around to pass in a callback queue, since we're being triggered directly from the tf2 buffer
* adding in y configuration test
* a little more realistic
* Don't add the request if the transform is already available.  Add some new tests
* working transformable callbacks with a simple (incomplete) test case
* cleaning up test setup
* check_v implemented and passing v test and multi tree test
* working toward multi configuration tests
* removing restructuring for it won't nest like I thought
* continuing restructuring and filling in test case setup
* restructuring before scaling
* Completely remove lookupLists().  canTransform() now uses the same walking code as lookupTransform().  Also fixed a bug in the static transform publisher test
* testing chaining in a ring
* test dataset generator
* more complicated test with interleaving static and dynamic frames passing
* static transform tested and working
* test in progress, need to unshelve changes.
* tests passing and all throw catches removed too\!
* move to tf2_ros completed. tests pass again
* merge tf2_cpp and tf2_py into tf2_ros
* merging and fixing broken unittest
* Got transform with types working in python
* A working first version of transforming and converting between different types
* removing unused datatypes
* removing include of old tf from tf2
* testing new argument validation and catching bug
* unit test of single link one to try to debug eitan's client bug
* working towards interpolation too
* A working version of a test case for the python buffer client
* merging
* adding else to catch uncovered cases, and changing time for easier use
* Adding a test for the python buffer client
* using permuter now and doing a,b,c to a,b,c, at three different times including 0
* Moving tf2_tests to test_tf2
* moving test to new package
* initial package created for testing tf2
