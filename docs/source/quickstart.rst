.. _quickstart:

Quickstart
==========
Getting started with CartesIO requires only a few setup steps, especially if you start from a
URDF/SRDF-based description of your robot. If you don't, please follow the relevant ROS documentation.
Once this is done, you just need to write a single configuration file to specify what Cartesian control
problem you actually want to solve.

Setting up the robot description
--------------------------------
For the robot description, we leverage on the ROS model, which requires you to write:

 - a **URDF** file with kinematic and dynamic properties of the robot
 - a **SRDF** file with semantic information attached to it, such as the division of your robot into separate
   kinematic chains, and the definition of a *homing* configuration

We pose the additional constraint that *floating base* robots should have their base link connected
to the ``world`` frame through a ``floating`` joint type.

Moreover, you should list all kinematic chains which make up the robot into a separate ``chains`` group inside
the SRDF. You can always refer to the `examples/` subfolder of the CartesIO ROS package,
which is accessed e.g. by typing ``roscd cartesian_interface/examples``

Setting up the problem description
----------------------------------
The problem description is a YAML file defining the control problem, i.e.:

 - which **tasks** do I want the robot to perform?
 - in which **priority** order?
 - under what **constraints**?

Please follow :ref:`this link<Problem>` to understand the basix syntax.

Writing an example launch file
------------------------------
For your commodity, it is better to wrap all the system setup in a single launch file.

.. code-block:: xml

    <launch>

        <param name="robot_description"
            textfile="$(find my_urdf_pkg)/my_robot.urdf"/>

        <param name="robot_description_semantic"
            textfile="$(find my_srdf_pkg)/my_robot.srdf"/>

        <param name="cartesian/problem_description"
            textfile="$(find my_cartesio_pkg)/my_robot_stack.yaml"/>

        <include file="$(find cartesian_interface)/launch/cartesio.launch">
            <!-- Control frequency (Hz) -->
            <arg name="rate" value="100.0"/>

            <!-- Spawn RviZ interacrive markers -->
            <arg name="markers" value="true"/>

            <!-- Set to false for fixed base  -->
            <arg name="is_model_floating_base" value="true"/>
        </include>

    </launch>

You can immediately start controlling your robot by executing this simple launch file!
It is now possible to interact with you Cartesian controller directly to the auto-generated
ROS API (``rostopic list``, ``rosservice list``), or through our C++/Python Client Library

.. _Tutorial:

Get started with the tutorial
-----------------------------
For most applications, you will interact with CartesIO through ROS. This can be conveniently
done either via raw topics, services, and actions (see :ref:`this page<RosApi>`), or by using
a C++/Python clent library. The usage of the Python Client Library is explained with a tutorial
that we've prepared, which will guide you through the available API.

To run the tutorial:
 - install the jupyter notebook (e.g. ``pip install notebook``, possibly with ``--user``
   if sudo permissions are not available)
 - open a web browser (e.g. chrome, firefox, ...)
 - ``roscd cartesian_interface/examples/python``
 - ``jupyter notebook ros_client_example.ipynb``
 - the notebook should display inside your browser

The tutorial notebook will look like in the following static document:

.. raw:: html

    <iframe src="_static/tutorial_nb.html" height="480px" width="100%"></iframe>

|
|

And will enable you to easily control the IIT-HHCM Coman robot!

.. figure:: tutorial_rviz.png
    :width: 90%
    :align: center

    Visualization of the IK solution in Rviz, with interactive markers.
