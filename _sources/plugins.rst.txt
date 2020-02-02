.. _Expand:

How to expand CartesIO
======================
CartesIO supports only a limited set of tasks and constraints. These are usually enough for
setting up most control problems; however, special tasks might be needed for implementing more
"exotic" behaviors. For this reason, CastesIO is designed to be **highly customizable** at almost
all its architectural layers; the user can implement:

 - a **new controller**, which replaces the OpenSot-based velocity level inverse kinematics.
   For instance, a custom solver can be implemented from scratch, but remaining compatible with all the
   CartesIO interfaces and utilities. New controllers can then dynamically loaded
   with the ``load_controller`` ROS service.

 - A **new task/constraint**, which is added to the library of supported CartesIO APIs. This way, we can
   a programmatic API, and a ROS interface to a new kind of task.

 - When a new task/constraint is developed, the **OpenSot support** can be quickly implemented as well,
   in order to have it inside the default OpenSot-based velocity level inverse kinematics controller.

The header files in ``<cartesian_interface/sdk/...>`` can be conveniently employed to re-use existing componens
of the CartesIO framework. For instance, a specialization of a ``Cartesian`` task
(such as a center-of-mass stabilizer) does not need to re-implement the whole Cartesian task API. All it needs
to do is to properly inherit from the proper classes.


.. toctree::
   :maxdepth: 1

   plugins_task
   plugins_ros
   plugins_sot
