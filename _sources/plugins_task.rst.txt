Adding a new task/constraint type
=================================

.. contents:: :local:

(1) Define an abstract interface
--------------------------------
First, we will just define the programmatic API of our task. This is done by defining a
**pure abstract class** with no constructor or implementation, just pure virtual methods.
This class will inherit from ``TaskDescription`` in order to be recognised as a task,
or from ``ConstraintDescription`` if it's a constraint.

.. code-block:: c++


    #ifndef ANGULARMOMENTUM_H
    #define ANGULARMOMENTUM_H

    #include <cartesian_interface/sdk/problem/Task.h>

    namespace XBot { namespace Cartesian {

    /**
     * @brief The AngularMomentum class models the abstract
     * interface for the task.
     *
     * NOTE: it is mandatory to inherit **virtually** from TaskDescription
     */
    class AngularMomentum : public virtual TaskDescription
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(AngularMomentum)

        /* Define the task API (pure virtual methods) */
        virtual Eigen::Vector3d getReference() const = 0;
        virtual void setReference(const Eigen::Vector3d& lref) = 0;

    };


(2) Declare an implementation class
-----------------------------------

.. code-block:: c++

    /**
     * @brief The AngularMomentumImpl class implements the abstract
     * interface
     */
    class AngularMomentumImpl : public TaskDescriptionImpl,
                                public virtual AngularMomentum
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(AngularMomentumImpl)

        /* The task implementation constructor signature must be
         * as follows */
        AngularMomentumImpl(YAML::Node task_node,
                            Context::ConstPtr context);

        /* Implement the task API */
        Eigen::Vector3d getReference() const override;
        void setReference(const Eigen::Vector3d& lref) override;

        /* Customize update, reset and log */
        void update(double time, double period) override;
        void reset() override;
        void log(MatLogger2::Ptr logger, bool init_logger, int buf_size) override;

    private:

        static constexpr double REF_TTL = 1.0;

        Eigen::Vector3d _lref;
        double _ref_timeout;
    };

    } }

    #endif // ANGULARMOMENTUM_H


(3) Implement all methods
-------------------------

The constructor signature must match the following one: every task is constructed by CartesIO
in a uniform way, with the relevant YAML node from the problem description file, and a const
share pointer to a context object.
You must call the ``TaskDescriptionImpl`` constructor additionally specifying

 - the task name (a specific name, different for all instances, e.g. "visual_servo_left_hand")
 - the task size (this will set the weight and indices size)

The constructor is also the place where you parse custom parameters from the given YAML.
The base class will do the same jobs for all *common Task parameters*.


.. code-block:: c++

    #include "AngularMomentum.h"

    #include "fmt/format.h"

    using namespace XBot::Cartesian;


    AngularMomentumImpl::AngularMomentumImpl(YAML::Node task_node,
                                             Context::ConstPtr context):
        TaskDescriptionImpl(task_node,
                            context,
                            "AngularMomentum",
                            3),
        _lref(0,0,0),
        _ref_timeout(-1)
    {
        /* Here you can parse custom YAML fields from task_node */

    }

In this example, we implement a *reference timeout* as reference management logic. This way,
if the user stops providing references, the desired value drops to zero automatically.

.. code-block:: c++

    Eigen::Vector3d AngularMomentumImpl::getReference() const
    {
        return _lref;
    }

    void AngularMomentumImpl::setReference(const Eigen::Vector3d& lref)
    {
        _lref = lref;
        _ref_timeout = getTime() + REF_TTL;
    }


.. note::

    All methods which are overridden from the base class ``TaskDescriptionImpl``
    must call the base class method!

.. code-block:: c++

    void XBot::Cartesian::AngularMomentumImpl::update(double time, double period)
    {
        // call base class
        TaskDescriptionImpl::update(time, period);

        // if the last reference has expired, the set it to zero
        if(time > _ref_timeout) _lref.setZero();

    }

    void XBot::Cartesian::AngularMomentumImpl::reset()
    {
        // call base class
        TaskDescriptionImpl::reset();

        _lref.setZero();
    }

    void XBot::Cartesian::AngularMomentumImpl::log(MatLogger2::Ptr logger,
                                                   bool init_logger,
                                                   int buf_size)
    {
        // call base class
        TaskDescriptionImpl::log(logger, init_logger, buf_size);

        if(init_logger)
        {
            logger->create(getName() + "_ref", 3, 1, buf_size);
            return;
        }

        logger->add(getName() + "_ref", _lref);
    }

(4) Register the plugin
-----------------------

.. code-block:: c++

    CARTESIO_REGISTER_TASK_PLUGIN(AngularMomentumImpl, AngularMomentum)


.. note::

    All ``*REGISTER*`` macros take as second argument the task name that will be specified
    by users inside the ``type`` yaml field. In this case, the OpenSot adapter will
    correspond to a task with ``type: AngularMomentum``

