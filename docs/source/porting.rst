Porting code from v1.x
======================
Some of the differences w.r.t. version 1.x are summarized hereafter.

General design
--------------

- Almost all functionalities of CartesIO are now provided **task-wise**, rather than from
  a global ``CartesianInterface`` object as before. The old API has been preserved for
  backward compatibility. A task object is obtained by name with the ``getTask()`` method,
  and is then dynamically casted with ``std::dynamic_pointer_cast``.
- The most part of the implementation has been moved to task-specific classes implementing

    - reference management (e.g. time-out, filtering, trajectory generation, ...)
    - ROS API
    - translation into OpenSoT tasks/constraints
    - ...
- A ``Subtask`` task type has been introduced to distribute parts of a task over different
  priorities
- Richer API
- The ``ControlMode`` property which could task values among ``Position``, ``Velocity``, ``Disabled``
  is now splitted into

    - an ``ActivationState`` which is defined for all tasks, and can be either ``Enabled`` or
      ``Disabled``
    - a ``ControlMode`` which is defined for Cartesian tasks only, and can be either ``Position`` or
      ``Velocity``


ROS
---

- ``task_name/state`` topic has been renamed to ``task_name/current_reference``
- ``world_odom`` TF frame has been removed, and the standard ``world`` is used instead



C++
---

- The ROS Client has been renamed, from ``RosImpl`` to ``RosClient``. The same applies to
  the header file name.
- A new ``Context`` object has been introduced to gather together a struct of ``Parameters``
  (among which the control rate), and the ``ModelInterface``
- Almost all constructors which would take a ``ModelInterface`` an input, now take a
  ``Context`` shared pointer
- Implementations of ``CartesianInterfaceImpl`` are now created with the static factory
  ``::MakeInstance``, rather than manually looking for the shared object and invoking
  its ``extern "C"`` factory method by name


Python
------

TBD
