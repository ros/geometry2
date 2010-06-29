Kinematic Chains
----------------

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


.. class:: Joint

  (jcb) these are odd - have to decide by hand what they are:

  * JointType
  * None
  * RotX
  * RotY
  * RotZ
  * TransX
  * TransY
  * TransZ

  .. method:: getType() -> None

  .. method:: getTypeName() -> None

  .. method:: pose() -> None

  .. method:: twist() -> None

.. class:: Segment

  .. method:: getFrameToTip() -> None

  .. method:: getJoint() -> None

  .. method:: pose() -> None

  .. method:: twist() -> None

.. class:: Chain

  .. method:: addChain(chain) -> None

    :param chain: chain to add
    :type chain: :class:`Chain`

    Adds a complete chain to the end of the chain The added chain is copied

  .. method:: addSegment(segment) -> None

    :param segment: segment to add
    :type segment: :class:`Segment`

    Adds a new segment to the end of the chain

  .. method:: getNrOfJoints() -> int

    Request the total number of joints in the chain. 

  .. method:: getNrOfSegments() -> int

    Request the total number of segments in the chain. 

  .. method:: getSegment(nr) -> segment

    :param nr: segment number, starting at zero
    :type nr: int
    :rtype: :class:`Segment`

    Request the nr'd segment of the chain. 
