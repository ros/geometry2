Geometric Primitives
--------------------

* :ref:`genindex`
* :ref:`search`

.. module:: PyKDL

Vector
^^^^^^

.. class:: Vector

  The Vector class describes both a 3D vector and a 3D point in space. This class supports operators + - += -= on other vectors,  and operators * / *= /= on doubles. The elements of the vector can be accessed using the [] operator from 0:2.


  .. method:: __init__()

    Empty constructor defaults to 0, 0, 0

  .. method:: __init__(x, y, z)

    Constructor with x, y, z

    :param x: x-component
    
    :type x: double

    :param y: y-component
    
    :type y: double

    :param z: z-component
    
    :type z: double

  .. method:: Norm() -> double

    Return the norm of the vector

  .. method:: Normalize() -> double

    Normalize the vector in place, and return the norm of the vector

  .. method:: ReverseSign() -> None

    Reverses the sign of the vector

  .. method:: Zero() -> None

    Sets all elements in the vector to zero

  .. method:: x() -> double

    Return the x component of the vector

  .. method:: y() -> double

    Return the y component of the vector

  .. method:: z() -> double

    Return the z component of the vector


Static functions:

  .. function:: dot(v1, v2) -> double

    Returns the dot product of two vectors

    :param v1: the first vector

    :type v1: :class:`Vector`

    :param v2: the second vector

    :type v2: :class:`Vector`

  .. function:: operator*(v1, v2) -> Vector

    Returns the cross producs of two vectors

    :param v1: the first vector

    :type v1: :class:`Vector`

    :param v2: the second vector

    :type v2: :class:`Vector`



Rotation
^^^^^^^^

.. class:: Rotation

  This class represents a 3D orientation in space. The internal representaion is a 3x3 rotation matrix. The elements of this matrix can be accessed using the [] operator with range 0:3, 0:3.

  .. method:: __init__()

    Emtpy constructor defaults to identity rotation

  .. method:: __init__(Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz)

    Constructor specifying rows of rotation matrix with 9 doubles

  .. method:: __init__(x, y, z)
    
    Constructor specifying rows of rotation matrix with 3 Vectors

    :param x: the first row of the rotation matrix

    :type x: :class:`Vector`

    :param y: the second row of the rotation matrix

    :type y: :class:`Vector`

    :param z: the third row of the rotation matrix

    :type z: :class:`Vector`

  .. method:: DoRotX(angle) -> None

    Apply a rotation around the x-axis with angle

    :param angle: the angle to rotate

    :type angle: double

  .. method:: DoRotY(angle) -> None

    Apply a rotation around the y-axis with angle

    :param angle: the angle to rotate

    :type angle: double

  .. method:: DoRotZ(angle) -> None

    Apply a rotation around the z-axis with angle

    :param angle: the angle to rotate

    :type angle: double

  .. method:: GetEulerZYX() -> (z, y, x)

   Returns the z, y, x Euler angles that describe this rotation. First a rotation around the z-axis, then around the rotated y-axis, and finally around the rotated x-axis.

  .. method:: GetEulerZYZ() -> (z1, y, z2)

   Returns the z, y, z Euler angles that describe this rotation. First a rotation around the z-axis, then around the rotated y-axis, and finally around the rotated z-axis.

  .. method:: GetQuaternion() -> (x, y, z, w)

   Returns the x, y, z, w normalized quaternion that describes this rotation

  .. method:: GetRPY() -> (r, p, y)

   Returns the r, p, y rotations around fixed axis that describe this rotation. First a rotation around the x-axis, then a rotation around the original y-axis, and finally a rotation around the original z-axis

  .. method:: GetRot() -> axis

   Returns a vector with the direction of the equivalent axis and its norm the angle. THis method returns the axis as a :class:`Vector`

  .. method:: GetRotAngle() -> (angle, axis)

   Returns the rotation angle around the equivalent axis. This method returns the angle as a double, and the rotation axis as a :class:`Vector`

  .. method:: Inverse() -> Rotation

   Returns the inverse rotation (this is also the transpose of the rotation matrix)


  .. method:: Rot2() -> None

  .. method:: SetInverse() -> None

  .. method:: UnitX() -> None

  .. method:: UnitY() -> None

  .. method:: UnitZ() -> None

  .. method:: operator*(Vector) -> Vector

   Changes the reference frame of a :class:`Vector`. The norm of the vector does not change.

  .. method:: operator*(Twist) -> Twist

   Changes the refenrece frame of a :class:`Twist`

  .. method:: operator*(Wrench) -> Wrench

   Changes the refenrece frame of a :class:`Wrench`

Static functions:

  .. function:: Identity() -> Rotation

   Constructs an identity rotation 

  .. function:: Quaternion(x, y, z, w) -> Rotation

   Constructs a rotation from an x, y, z, w quaternion descripion

  .. function:: Rot(axis, angle) -> Rotation

   Constructs a rotation from a rotation of angle around axis

   :param axis: the axis to rotate around

   :type axis: :class:`Vector`

   :param angle: the angle to rotate

   :type angle: double

  .. function:: RotX(angle) -> Rotation

   Constructs a rotation of angle around the x-axis

  .. function:: RotY(angle) -> Rotation

   Constructs a rotation of angle around the y-axis

  .. function:: RotZ(angle) -> Rotation

   Constructs a rotation of angle around the z-axis

  .. function:: EulerZYX(z, y, x) -> Rotation

   Constructs a rotation by first applying a rotation of z around the z-axis, then a rotation of y around the new y-axis, and finally a rotation of x around the new x-axis

  .. function:: EulerZYZ(z1, y, z2) -> Rotation

   Constructs a rotation by first applying a rotation or z1 around the z-axis, then a rotation  of y around the new y-axis, and finally a rotation of z2 around the new z-axis

  .. function:: RPY(r, p, y) -> Rotation

   Constructs a rotation by first applying a rotation of r around the x-axis, then a rotation of p around the original y-axis, and finally a rotation of y around the original z-axis


Frame
^^^^^^^^^^^^^^^

.. class:: Frame

  .. attribute:: M

   This is the :class:`Rotation` of the frame

  .. attribute:: p

   This is the :class:`Vector` of the frame

  .. method:: __init__()

   Construct an identity frame

  .. method:: __init__(pos, rot)

   Construct a frame from a vector and a rotation

   :param pos: the position of the frame origin

   :type pos: :class:`Vector`

   :param rot: the rotation of the frame

   :type rot: :class:`Rotation`

  .. method:: __init__(pos)

   Construct a frame from a vector, with identity rotation

   :param pos: the position of the frame origin

   :type pos: :class:`Vector`

  .. method:: __init__(rot)

   Construct a frame from a rotation, with origin at 0, 0, 0

   :param rot: the rotation of the frame

   :type rot: :class:`Rotation`

  .. method:: Integrate(twist, frequency) -> None

   This frame is integrated into an updated frame with sample frequence, using first order integration

   :param twist: this twist is represented with respect to the current frame

   :type twist: :class:`Twist`

   :param frequency: the sample frequency to update this frame

   :type frequence: double

  .. method:: Inverse() -> Frame

   Returns the inverse of the frame

  .. method:: operator*(Vector) -> Vector

   Changes both the reference frame and the reference point of a :class:`Vector`. Use this operator when the vector represents a point

  .. method:: operator*(Twist) -> Twist

   Changes bothe the refenrece frame and the referece point of a :class:`Twist`

  .. method:: operator*(Wrench) -> Wrench

   Changes both the refenrece frame and the reference point of a :class:`Wrench`


Static functions:

  .. function:: Identity() -> Frame
  
   Constructs an identity frame

  .. function:: HD(a, alpha, d, theta) -> Frame

   Constructs a transformationmatrix T_link(i-1)_link(i) with the Denavit-Hartenberg convention as described in the original publictation: Denavit, J. and Hartenberg, R. S., A kinematic notation for lower-pair mechanisms based on matrices, ASME Journal of Applied Mechanics, 23:215-221, 1955.      

  .. function:: DH_Craig1989(a, alpha, d, theta) -> Frame

   Constructs a transformationmatrix T_link(i-1)_link(i) with the Denavit-Hartenberg convention as described in the Craigs book: Craig, J. J.,Introduction to Robotics: Mechanics and Control, Addison-Wesley, isbn:0-201-10326-5, 1986.   

  .. function:: AddDelta(f, t, d) -> Frame

   Constructs a frame that is obtained by: starting from frame f, apply twist t, during time d

   :param f: the frame to start the integration from

   :type f: :class:`Frame`

   :param t: the twist to apply, represented in the same reference frame as f, and with reference point at the origin of f

   :type t: :class:`Twist`

   :param d: the duration to apply twist t

   :type d: double

  .. function:: diff(f1, f2, d) -> Twist

   Returns the twist that is needed to move from frame f1 to frame f2 in a time d. The resulting twist is represented in the same reference frame as f1 and f2, and has reference point at the origin of f1

   :param f1: the frame to start from

   :type f1: :class:`Frame`

   :param f2: the frame to end up in

   :type f2: :class:`Frame`

   :param d: the duration to apply the resulting twist

   :type d: double



Twist
^^^^^^^^^^^^^^^
.. class:: Twist

  .. method:: RefPoint() -> None

  .. method:: ReverseSign() -> None

  .. method:: Zero() -> None

  .. attribute:: rot

  .. attribute:: vel


Wrench
^^^^^^^^^^^^^^^
.. class:: Wrench

  .. method:: RefPoint() -> None

  .. method:: ReverseSign() -> None

  .. method:: Zero() -> None

  .. attribute:: force

  .. attribute:: torque




