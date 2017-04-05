^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_eigen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.15 (2017-01-24)
-------------------
* fixup `#186 <https://github.com/ros/geometry2/issues/186>`_: inline template specializations (`#200 <https://github.com/ros/geometry2/issues/200>`_)
* Contributors: Robert Haschke

0.5.14 (2017-01-16)
-------------------
* Add tf2_eigen conversions for Pose and Point (not stamped) (`#186 <https://github.com/ros/geometry2/issues/186>`_)
  * tf2_eigen: added conversions for Point msg type (not timestamped) to Eigen::Vector3d
  * tf2_eigen: added conversions for Pose msg type (not timestamped) to Eigen::Affine3d
  * tf2_eigen: new functions are inline now
  * tf2_eigen test compiling again
  * tf2_eigen: added tests for Affine3d and Vector3d conversion
  * tf2_eigen: added redefinitions of non-stamped conversion function to make usage in tf2::convert() possible
  * tf2_eigen: reduced redundancy by reusing non-stamped conversion-functions in their stamped counterparts
  * tf2_eigen: added notes at doTransform-implementations which can not work with tf2_ros::BufferInterface::transform
  * tf2_eigen: fixed typos
* Don't export local include dirs (`#180 <https://github.com/ros/geometry2/issues/180>`_)
* Improve documentation.
* Contributors: Jackie Kay, Jochen Sprickerhof, cwecht

0.5.13 (2016-03-04)
-------------------
* Added missing inline
* Added unit test
  - Testing conversion to msg forward/backward
* Added eigenTotransform function
* Contributors: Davide Tateo, boris-il-forte

0.5.12 (2015-08-05)
-------------------

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------
* fixing CMakeLists.txt from `#97 <https://github.com/ros/geometry_experimental/issues/97>`_
* create tf2_eigen.
* Contributors: Tully Foote, koji

