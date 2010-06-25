Geometric Primitives
--------------------


Available Types
^^^^^^^^^^^^^^^

.. class:: Vector

  .. method:: Norm() -> None

    This is a comment. :class:`Chain`, :meth:`Vector.Normalize`

  .. method:: Normalize() -> None

  .. method:: ReverseSign() -> None

  .. method:: Zero() -> None

  .. method:: x() -> None

  .. method:: y() -> None

  .. method:: z() -> None

.. class:: Rotation

  .. method:: DoRotX() -> None

  .. method:: DoRotY() -> None

  .. method:: DoRotZ() -> None

  .. method:: EulerZYX() -> None

  .. method:: EulerZYZ() -> None

  .. method:: GetEulerZYX() -> None

  .. method:: GetEulerZYZ() -> None

  .. method:: GetQuaternion() -> None

  .. method:: GetRPY() -> None

  .. method:: GetRot() -> None

  .. method:: GetRotAngle() -> None

  .. method:: Identity() -> None

  .. method:: Inverse() -> None

  .. method:: Quaternion() -> None

  .. method:: RPY() -> None

  .. method:: Rot() -> None

  .. method:: Rot2() -> None

  .. method:: RotX() -> None

  .. method:: RotY() -> None

  .. method:: RotZ() -> None

  .. method:: SetInverse() -> None

  .. method:: UnitX() -> None

  .. method:: UnitY() -> None

  .. method:: UnitZ() -> None

.. class:: Frame

  .. method:: DH() -> None

  .. method:: DH_Craig1989() -> None

  .. method:: Identity() -> None

  .. method:: Integrate() -> None

  .. method:: Inverse() -> None

  .. attribute:: M

  .. attribute:: p

.. class:: Twist

  .. method:: RefPoint() -> None

  .. method:: ReverseSign() -> None

  .. method:: Zero() -> None

  .. attribute:: rot

  .. attribute:: vel

.. class:: Wrench

  .. method:: RefPoint() -> None

  .. method:: ReverseSign() -> None

  .. method:: Zero() -> None

  .. attribute:: force

  .. attribute:: torque

(jcb) Not sure where these should go:

.. class:: FrameVel

  .. method:: GetFrame() -> None

  .. method:: GetTwist() -> None

  .. method:: Identity() -> None

  .. method:: Inverse() -> None

  .. attribute:: M

  .. method:: deriv() -> None

  .. attribute:: p

  .. method:: value() -> None

.. class:: Jacobian

  .. method:: columns() -> None

  .. method:: rows() -> None

.. class:: JntArray

  .. method:: columns() -> None

  .. method:: rows() -> None

.. class:: JntArrayVel

  .. method:: deriv() -> None

  .. attribute:: q

  .. attribute:: qdot

  .. method:: value() -> None

.. class:: Multiply

.. class:: MultiplyJacobian

.. class:: RotationVel

  .. method:: DoRotX() -> None

  .. method:: DoRotY() -> None

  .. method:: DoRotZ() -> None

  .. method:: Identity() -> None

  .. method:: Inverse() -> None

  .. attribute:: R

  .. method:: Rot() -> None

  .. method:: Rot2() -> None

  .. method:: RotX() -> None

  .. method:: RotY() -> None

  .. method:: RotZ() -> None

  .. method:: UnitX() -> None

  .. method:: UnitY() -> None

  .. method:: UnitZ() -> None

  .. method:: deriv() -> None

  .. method:: value() -> None

  .. attribute:: w

.. class:: TwistVel

  .. method:: GetTwist() -> None

  .. method:: GetTwistDot() -> None

  .. method:: RefPoint() -> None

  .. method:: ReverseSign() -> None

  .. method:: Zero() -> None

  .. method:: deriv() -> None

  .. method:: value() -> None

.. class:: VectorVel

  .. method:: Norm() -> None

  .. method:: ReverseSign() -> None

  .. method:: Zero() -> None

  .. method:: deriv() -> None

  .. attribute:: p

  .. attribute:: v

  .. method:: value() -> None

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`


