Cartesian task ROS API
======================

Contents:

.. contents:: :local:

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
Allows to set the task base link.

.. list-table:: Service description
   :widths: 8 30
   :header-rows: 0
   :stub-columns: 1


   * - Name
     - set_base_link
   * - Type
     - ``cartesian_interface::SetBaseLink``
   * - Request
     - .. code-block:: yaml

            string base_link
   * - Response
     - .. code-block:: yaml

            bool success
            string message


set_control_mode
^^^^^^^^^^^^^^^^
Allows to set the task control mode.

.. list-table:: Service description
   :widths: 8 30
   :header-rows: 0
   :stub-columns: 1


   * - Name
     - set_base_link
   * - Type
     - ``cartesian_interface::SetControlMode``
   * - Request
     - .. code-block:: yaml

            string control_mode
   * - Response
     - .. code-block:: yaml

            bool success
            string message

The ``control_mode`` field is a case-insensitive string which can take the following values:
*position*, or *velocity*.

set_safety_limits
^^^^^^^^^^^^^^^^^
Allows to set the task safety limits which are applied to reference signals,
in terms of maximum velocity and acceleration for both
the linear and angular component of the task.

.. list-table:: Service description
   :widths: 8 30
   :header-rows: 0
   :stub-columns: 1


   * - Name
     - set_base_link
   * - Type
     - ``cartesian_interface::SetSafetyLimits``
   * - Request
     - .. code-block:: yaml

            float32 max_vel_lin
            float32 max_vel_ang
            float32 max_acc_lin
            float32 max_acc_ang
   * - Response
     - .. code-block:: yaml

            bool success
            string message

Only values which are greater than zero are actually taken into consideration. Non-positive values
are discarded.

Topics
------

reference (input)
^^^^^^^^^^^^^^^^^
Input topic for user-provided Cartesian references. Type: ``geometry_msgs/PoseStamped``.

velocity_reference (input)
^^^^^^^^^^^^^^^^^^^^^^^^^^
Input topic for user-provided Cartesian velocity references. Type: ``geometry_msgs/TwistStamped``.

current_reference (output)
^^^^^^^^^^^^^^^^^^^^^^^^^^
Output topic containing the most recent Cartesian reference tracked by the controller.
Type: ``geometry_msgs/PoseStamped``.

current_velocity_reference (output)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Output topic containing the most recent Cartesian velocity reference tracked by the controller.
Type: ``geometry_msgs/TwistStamped``.

Actions
-------

reach
^^^^^
This action allows to command a reaching motion to a final frame in a given time, possibly
passing through user-defined waypoints.

.. list-table:: Action description
   :widths: 8 30
   :header-rows: 0
   :stub-columns: 1


   * - Name
     - reach
   * - Type
     - ``cartesian_interface::ReachPoseAction``
   * - Goal
     - .. code-block:: yaml

            geometry_msgs/Pose[] frames
            float32[] time
            bool incremental
       | This field allows to specify waypoints along with their respective times
       | (absolute, w.r.t. trajectory start). The incremental flag, if set to ``true``,
       | allows to specify waypoints w.r.t. the starting pose of the robot.
   * - Feedback
     - .. code-block:: yaml

            geometry_msgs/PoseStamped current_reference
            geometry_msgs/PoseStamped current_pose
            int32 current_segment_id # waypoint currently being processed
   * - Result
     - .. code-block:: yaml

            geometry_msgs/Pose final_frame
            float32 position_error_norm
            float32 orientation_error_angle
