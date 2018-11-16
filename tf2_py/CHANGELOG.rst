^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.20 (2018-11-16)
-------------------

0.5.19 (2018-11-06)
-------------------
* fix translation vs rotation typo
  Fixes `#324 <https://github.com/ros/geometry2/issues/324>`_
* Add python3.7 compatibility.
* Contributors: Hans Gaiser, Tully Foote

0.5.18 (2018-07-10)
-------------------

0.5.17 (2018-01-01)
-------------------
* Merge pull request `#266 <https://github.com/ros/geometry2/issues/266>`_ from randoms/indigo-devel
  fix METH_OLDARGS is no longer supported error in python3
* Merge pull request `#260 <https://github.com/ros/geometry2/issues/260>`_ from randoms/indigo-devel
  fix python3 import error
* Merge pull request `#257 <https://github.com/ros/geometry2/issues/257>`_ from delftrobotics-forks/python3
  Make tf2_py python3 compatible again
* Use string conversion from python_compat.h.
* Contributors: Maarten de Vries, Tully Foote, randoms

0.5.16 (2017-07-14)
-------------------
* fix memory leak calling Py_DECREF for all created PyObject
* replaced dependencies on tf2_msgs_gencpp by exported dependencies
* Relax strict type checks at setTransform to only check for members (`#221 <https://github.com/ros/geometry2/issues/221>`_)
* expose deprecated methods in tf2_py API to support better backwards compatibility. Fixes `#206 <https://github.com/ros/geometry2/issues/206>`_
* Contributors: Christopher Wecht, Sergio Ramos, Tully Foote, alex

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Improve tf compatibility (`#192 <https://github.com/ros/geometry2/issues/192>`_)
  getLatestCommonTime() is needed to implement the TF API.
  See `ros/geometry#134 <https://github.com/ros/geometry/issues/134>`_
* Add missing type checks at Python/C++ tf2 transform interface `#159 <https://github.com/ros/geometry2/issues/159>`_ (`#197 <https://github.com/ros/geometry2/issues/197>`_)
* Make tf2_py compatible with python3. (`#173 <https://github.com/ros/geometry2/issues/173>`_)
  * tf2_py: Use PyUnicode objects for text in python3.
  * tf2_py: Make module initialization python3 compatible.
  * tf2_py: Fix type definition for python3.
  * tf2_py: Move and rename PyObject_BorrowAttrString.
* Contributors: Maarten de Vries, Timo Röhling, alex

0.5.13 (2016-03-04)
-------------------

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

0.5.7 (2014-12-23)
------------------

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------

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
* adding support for static transforms in python listener. Fixes `#46 <https://github.com/ros/geometry_experimental/issues/46>`_
* Contributors: Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------
* tf2_py: Fixes warning, implicit conversion of NULL

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
