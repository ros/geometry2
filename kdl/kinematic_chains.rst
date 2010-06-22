Kinematic Chains
----------------

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
