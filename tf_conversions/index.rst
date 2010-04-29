tf_conversions
==============

.. toctree::
    :maxdepth: 2

PoseMath
--------

The posemath module is useful for working with poses from a variety of
sources: from :meth:`tf.Transformer.lookupTransform`, from :mod:`opencv`
or from ROS messages.  It has utility functions to convert between these
types and the :class:`PyKDL.Frame` pose representation.

.. doctest::
    :options: -ELLIPSIS, +NORMALIZE_WHITESPACE

    >>> from geometry_msgs.msg import Pose
    >>> import tf_conversions.posemath as pm
    >>> msg = Pose()
    >>> msg.position.x = 7.0
    >>> msg.orientation.w = 1.0
    >>> p = pm.fromMsg(msg)
    >>> print pm.toMsg(p)
    position: 
      x: 7.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

.. automodule:: tf_conversions.posemath
    :members: fromTf, fromMsg, toMsg, fromMatrix, toMatrix, fromCameraParams

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

