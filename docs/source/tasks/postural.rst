Postural task
=============
The postural task has two peculiarities w.r.t. a generic *Task*. First, subtasks of a Postural task
are not specified with the ``indices`` field, but only through the ``enabled_joints`` or
``disabled_joint`` fields. Second, the ``weight`` field can be expressed as a **map** between
**joint_name** and **weight value**-

Example
-------

.. code-block:: yaml

    MyPostural:
        type: Postural
        lambda: 0.01
        weight:
            joint1: 10
            joint2: 10
            # all missing joints will be given weight equal 1.0
        disabled_joints:
            - joint3
            - joint4
