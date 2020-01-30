Cartesian task
==============
A *Cartesian* task is specified by the ``type: Cartesian`` property. Its parameters
are described below.

distal_link
^^^^^^^^^^^
specifies the robot link to be controlled (mandatory, must be a valid robot link)

base_link
^^^^^^^^^
specifies the robot link w.r.t. which the ``distal_link`` is controlled
(optional, defaults to ``world``)

use_local_subtasks
^^^^^^^^^^^^^^^^^^
selects whether subtasks are defined w.r.t. the distal_link
local frame, or w.r.t. the global (base_link) frame (optional, defaults to ``false``)

orientation_gain
^^^^^^^^^^^^^^^^
specifies the ratio between the orientation and position feedback;
for instance, a value of ``orientation_gain: 2.0`` means that an orientation error of
2 rad is recovered in the same time as a position error of 1 m.

Example
-------

.. code-block:: yaml

    MyCartesianTask:
        name: left_arm
        type: Cartesian
        distal_link: arm1_8
        base_link: pelvis
        use_local_subtasks: true
        orientation_gain: 5.0
