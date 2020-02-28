Programmatic API (Python)
=========================

Basic usage
-----------
.. code-block:: python

    from cartesian_interface.pyci_all import *
    cli = pyci.CartesianInterfaceRos()
    task = cli.getTask('MyTask')
    # use task...

Check the :ref:`tutorial<Tutorial>` for an extensive example of the Python API!
The Python API closely matches the C++ API, for which you can check a reference by following
:ref:`this link<cppref>`.


Python interactive session
--------------------------
At any time, it is possible to run an interactive Python session which is
completely pre-configured for CartesIO, by running ``rosrun cartesian_interface interactive_client.py``.
Variables for all defined tasks will be dynamically created for the best convenience of use.

Example:

.. code-block:: bash

    $ rosrun cartesian_interface interactive_client.py

    Python 2.7.12 (default, Oct  8 2019, 14:14:10)
    Type "copyright", "credits" or "license" for more information.

    IPython 5.8.0 -- An enhanced Interactive Python.
    ?         -> Introduction and overview of IPython's features.
    %quickref -> Quick reference.
    help      -> Python's own help system.
    object?   -> Details about 'object', use 'object??' for extra details.

    [success] Successfully added Cartesian task with
       BASE LINK:   base_link
       DISTAL LINK: TCP
    [success] Successfully added postural task 'Postural'
    [success] Successfully added postural task 'MinVel'
    [success] Successfully added task 'JointLimits' with type 'Task'
    [success] Successfully added task 'VelocityLimits' with type 'Task'

    # TCP, Postural, etc... variables ready to use!
    In [1]: TCP.getPoseReference()
    Out[1]:
    (translation: [0.7581,      0, 0.1223]
     rotation   : [     0, 0.8415,      0, 0.5403],
     array([ 0.,  0.,  0.,  0.,  0.,  0.]),
     array([ 0.,  0.,  0.,  0.,  0.,  0.]))

