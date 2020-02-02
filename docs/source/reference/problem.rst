Problem Description API Reference
=================================

The header files in ``<cartesian_interface/problem/..>`` contain the API definition for all natively supported
tasks. These are abstract classes which are inherited by:
 - implementation classes, which define the runtime behavior of tasks (e.g. reference management,
   safety features, logging, ...)
 - client APIs, such as ROS client classes


.. toctree::
   :maxdepth: 1

   task
   cartesian
