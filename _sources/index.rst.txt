.. CartesIO documentation master file, created by
   sphinx-quickstart on Thu Jan 30 12:45:00 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to CartesI/O's documentation!
=====================================
A novel Cartesian control framework with a focus on online
control of multi-chained, hyper-redundant floating-base
robots!

Features
--------
.. image:: features.png
    :width: 90%
    :align: center

|
|

- **multiple task** specification
- ability to enforce soft **priorities** as well as hard priorities
  between tasks
- ability to specify **constraints** in the task execution
- small computation time (suitable for **online execution**)
- possibility to execute inside a **real-time** thread in order
  to reduce delays and jitter
- ease of configuration and use, **quick setup** time and
  ready to use control tools
- parametrized with **standard** description formats (e.g.
  URDF) in order to support multiple platforms
- handling of floating base robots
- heavily **customizable** through plugins, at almost all layers

How to install
--------------
Because CartesIO 2.0 is still in the works, it can only be compiled from source.
This option is though only available to IIT-HHCM fellows and employees.
Pre-build binaries will be shipped soon!

Cite our work
-------------
..with the bibtex below!

.. code-block:: latex

    @inproceedings{laurenzi19cartesio,
    author={A. {Laurenzi} and E. M. {Hoffman} and L. {Muratore} and N. G. {Tsagarakis}},
    booktitle={2019 International Conference on Robotics and Automation (ICRA)},
    title={{CartesI/O: A ROS Based Real-Time Capable Cartesian Control Framework}},
    year={2019},
    pages={591-596},
    doi={10.1109/ICRA.2019.8794464},
    ISSN={1050-4729},
    month={May},}

Contents
--------
Start from :ref:`here<quickstart>`!

.. toctree::
   :maxdepth: 2
   :caption: Design and usage

   quickstart
   design
   problemdesc
   rosapi
   progr_cpp
   progr_py
   plugins


.. toctree::
   :maxdepth: 2
   :caption: API reference

   reference/problem
   reference/trajectory
   reference/sdk


.. Indices and tables
.. ==================

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`

