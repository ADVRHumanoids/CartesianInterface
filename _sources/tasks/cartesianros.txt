Cartesian task ROS API
======================

Services:
 - `get_cartesian_task_properties`_
 - `set_base_link`_
 - `set_control_mode`_
 - `set_safety_limits`_

Topics:
 - `reference`_
 - `velocity_reference`_
 - `current_reference`_
 - `current_velocity_reference`_

Actions:
 - `reach`_

Services
--------

get_cartesian_task_properties
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Returns all cartesian task-specific properties.

.. list-table:: Service description
   :widths: 8 30
   :header-rows: 0
   :stub-columns: 1


   * - Name
     - get_cartesian_task_properties
   * - Type
     - ``cartesian_interface::GetCartesianTaskInfo``
   * - Request
     - --
   * - Response
     - .. code-block:: yaml

            string  base_link
            string  distal_link
            string  control_mode        # either "Position" or "Velocity"
            string  state               # either "Reaching" or "Online"
            bool    use_local_subtasks
            float64 max_vel_lin
            float64 max_vel_ang
            float64 max_acc_lin
            float64 max_acc_ang

set_base_link
^^^^^^^^^^^^^

set_control_mode
^^^^^^^^^^^^^^^^

set_safety_limits
^^^^^^^^^^^^^^^^^


Topics
------

reference
^^^^^^^^^

velocity_reference
^^^^^^^^^^^^^^^^^^

current_reference
^^^^^^^^^^^^^^^^^

current_velocity_reference
^^^^^^^^^^^^^^^^^^^^^^^^^^

Actions
-------

reach
^^^^^
