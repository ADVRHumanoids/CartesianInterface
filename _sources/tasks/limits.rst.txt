JointLimits
===========
A *JointLimits* constraint is specified by the ``type: JointLimits`` property. Its parameters
are described below.

limits
^^^^^^
map field, where the key represents a valid joint name, and the value is a ``[qmin, qmax]`` pair.
The field is optional, and all missing joints take their limits from the URDF file.

bound_scaling
^^^^^^^^^^^^^
Scaling factor for the joint range, w.r.t. the the center of the range. For example, with a
``bound_scaling: 2.0`` parameter, the range ``[0, 2]`` will be transformed into ``[-1, 3]``.
The field is optional, and the default value is 1.0

Example
-------

.. code-block:: yaml

    MyJointLims:
        type: JointLimits
        bound_scaling: 0.9 # 90% of the specified range
        limits:
            j_arm1_1: [-1, 1] # custom value
            j_arm1_7: [0, 0]  # lock joint to 0
            # all other joints take the URDF limits


VelocityLimits
==============
A *VelocityLimits* constraint is specified by the ``type: VelocityLimits`` property. Its parameters
are described below.

limits
^^^^^^
Either one of the following:

 - map field, where the key represents a valid joint name, and the value is a ``qdot_max`` positive float.
 - scalar field, which overwrites all limits from URDF to the given constant value
The field is optional, and all missing joints take their limits from the URDF file.

bound_scaling
^^^^^^^^^^^^^
Scaling factor for the joint velocity limit.
The field is optional, and the default value is 1.0

Example
-------

.. code-block:: yaml

    MyVelLims:
        type: VelocityLimits
        bound_scaling: 0.9 # 90% of the velocity limit
        limits:
            j_arm1_1: 1 # custom limit
            # all other joints take the URDF limits

