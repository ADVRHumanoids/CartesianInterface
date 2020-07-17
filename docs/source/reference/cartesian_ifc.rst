.. _cartesian_ifc:

CartesianInterface API Reference
================================

The header files ``CartesianInterface.h`` and ``CartesianInterfaceImpl.h``
contain the API definition for the main CartesIO class, which users can load
via the provided factory method ``CartesianInterfaceImpl::MakeInstance()``.
After loading, the recommended way to interact with the defined tasks
and constraint is to use their own specific APIs, as
obtained via the getTask() method.


.. doxygenclass:: XBot::Cartesian::CartesianInterface
    :members:

.. doxygenclass:: XBot::Cartesian::CartesianInterfaceImpl
    :members:

