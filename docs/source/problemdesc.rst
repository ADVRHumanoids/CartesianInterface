.. _Problem:

Writing a problem description
=============================

YAML basic structure
--------------------
The basic structure of a problem description file is comprised by

- a **stack** field specifying a list of tasks *for each priority layer*.
  Tasks are declared as *labels* which are later defined at the bottom of the file.
- a **constraints** field specifying bounds for the underlying optimization. Again,
  these are declared as textual labels to be defined after.
- a task/constraint **definition** part where each textual label declared before is actually
  defined

For example, a simple stack for a bimanual, redundant manipulation platform could be:

.. code-block:: yaml

    ## stack of tasks definition ##

    stack:
        - ["LeftArm", "RightArm"]
        - ["Postural"]
    constraints: ["JointLimits"]

    ## task and constraint definition ##

    JointLimits:
        type: "JointLimits"

    LeftArm:
        type: "Cartesian"
        distal_link: "arm1_8"

    RightArm:
        type: "Cartesian"
        distal_link: "arm2_8"

    Postural:
        type: "Postural"


Task common parameters
----------------------

All tasks share a list of common parameters, described below.

type
^^^^
is the literal name of the task type; a list of natively-supported
types is available :ref:`here<SupportedTasks>`, whereas :ref:`this section<Expand>`
contains details on how to create **custom types**

name
^^^^
is the name of the task, which will be used to interact with the task itself,
both from the programmatic API (C++/Python), and from the ROS api (topics, services, ...).
The ``name`` field is optional, the default depending on the task type.

active
^^^^^^
is a boolean specifying whether the task should be active right from creation,
or not. It defaults to ``true``.

lambda
^^^^^^
is the value of the feedback gain for position tracking, as a floating point number
ranging from 0 to 1. Defaults to 1.0.

lambda2
^^^^^^^
is the value of the feedback gain for velocity tracking (generally used by second-order controllerd),
as a floating point number ranging from 0 to 1. Defaults to -1, which means that the provided value
shound not be used. A reasonable default is therefore computed based on the value of ``łambda``.

weight
^^^^^^
is used to give different *soft priority* to the task *degrees of freedom*, also w.r.t.
other tasks at the same priority level. It must be either a scalar (e.g. ``weight: 10.0``) or a vector
with as many elements as the task size (e.g. ``weight: [1., 1., 1., 4., 4., 4.]`` for a 6D task). Defaults to
the identity.

indices
^^^^^^^
extract a subtask from the given vector of indices. For instance, ``indices: [0, 1, 2]`` extracts
the position task from a *Cartesian* task. Defaults to ``indices: [0, 1, ..., size-1]``)

disabled_joints
^^^^^^^^^^^^^^^
is a vector of joints that should not be used to perform the task

enabled_joints
^^^^^^^^^^^^^^
provides a way to specify the disabled_joints in terms of which joints will
be enabled

lib_name
^^^^^^^^
specifies the library with the definition of the custom task type (see :ref:`this section<Expand>`)

Task definition example
-----------------------

.. code-block:: yaml

    LeftArm:
        type: TaskTypeName
        name: left_arm
        active: true
        lambda: 0.1
        weight: 10.0
        indices: [0, 1, 2]
        lib_name: libCartesioTaskTypeName.so
        disabled_joints: [j_arm1_7, torso_yaw]

.. _SupportedTasks:

Supported task types
--------------------
.. toctree::
   :maxdepth: 1

   tasks/cartesian
   tasks/postural
   tasks/limits

Splitting tasks into subtasks
-----------------------------
The ``indices`` field is used to select a subtask from a given task. Sometimes,
the user wants to **split** a unique "base" class into multiple subtasks, usually
to assign different priorities to them. In this case, it is necessary that, for instance,
sending references to the "base" task will affect all subtasks.

CartesIO offers the ``Subtask`` type in order to address this need, as in the following example:

.. code-block:: yaml

    ComTask:
        type: "Com"
        weight: 10
        lambda: 0.05

    ComXY:
        type: "Subtask"
        task: "ComTask"
        indices: [0, 1]

    ComZ:
        type: "Subtask"
        task: "ComTask"
        indices: [2]

It is then possible to use the task API to (for instance) enable/disable single subtasks,
or change their weight. The same can of course be done on the "base" task. An example using
the Python API would be:

.. code-block:: python

    com = cli.getTask('ComTask')
    com_xy = cli.getTask('ComXY')
    com_z = cli.getTask('ComZ')

    com_z.setActivationState(pyci.ActivationState.Disabled) # disable only z dof
    com_xy.setWeight(np.eye(2) * 100.0) # set weight only for xy



Specifying tasks as constraints
-------------------------------
It is possible to turn tasks into constraints by simply listing them inside the
``constraints`` field.

