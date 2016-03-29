tf2_ros
=======

This is the Python API reference of the tf2_ros package.

To broadcast transforms using ROS:
- Call :function:`rospy.init` to initialize a node.
- Construct a :class:`tf2_ros.TransformBroadcaster`.
- Pass a `geometry_msgs::TransformStamped` message to `tf2_ros.TransformBroadcaster.sendTransform`.
  - Alternatively, pass a vector of `geometry_msgs.TransformStamped` messages.

Use StaticTransformBroadcaster for "latching" behavior for transforms that are not expected to change.

To listen for transforms using ROS:
- Construct a :class:`tf2_ros.Buffer`.
- Pass the :class:`tf2_ros.Buffer` to the constructor of :class:`tf2_ros.TransformListener`.
  - Optionally, pass a :class:`ros.NodeHandle` (otherwise TransformListener will connect to the node for the process).
  - Optionally, specify if the TransformListener runs in its own thread or not.
- Check if a transform is available with :function:`tf2_ros.Buffer.canTransform()`.
- Call :function:`tf2_ros.Buffer.lookupTransform()` to get the transform between two frames.


See http://wiki.ros.org/tf2/Tutorials for more detailed usage.


.. toctree::
    :maxdepth: 2

    tf2_ros

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
