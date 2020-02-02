Implementing OpenSot support for a custom task
==============================================

To implement the OpenSot support, you need to define a subclass of ``OpenSotTaskAdapter``.
You must implement:
 - a constructor with fixed signature, which checks that the provided
   `TaskDescription` object is of the proper type
 - a ``constructTask`` method which actually construct the OpenSot task instance
 - an optional ``initialize`` override if you need to perform some post-construction operation. The
   base class implementation will perform all generic task operations such as setting the lambda, weight,
   indices, active joint mask, ...
 - an optional ``update`` override to customize the control loop. The base class method will update lambda
   weight.

.. code-block:: c++

    #ifndef OPENSOTANGULARMOMENTUM_H
    #define OPENSOTANGULARMOMENTUM_H

    #include "AngularMomentum.h"
    #include <cartesian_interface/sdk/opensot/OpenSotTask.h>

    #include <OpenSoT/tasks/velocity/AngularMomentum.h>

    using AngularMomentumSoT = OpenSoT::tasks::velocity::AngularMomentum;

    namespace XBot { namespace Cartesian {

    class OpenSotAngularMomentum : public OpenSotTaskAdapter
    {

    public:

        OpenSotAngularMomentum(TaskDescription::Ptr task,
                               ModelInterface::ConstPtr model);

        virtual TaskPtr constructTask() override;

        virtual bool initialize() override;

        virtual void update(double time, double period) override;

    private:

        AngularMomentumSoT::Ptr _sot_angmom;
        AngularMomentum::Ptr _ci_angmom;


    };

    } }

    #endif // OPENSOTANGULARMOMENTUM_H


.. code-block:: c++

    #include "OpenSotAngularMomentum.h"

    #include <boost/make_shared.hpp>

    using namespace XBot::Cartesian;

    OpenSotAngularMomentum::OpenSotAngularMomentum(TaskDescription::Ptr task,
                                                   ModelInterface::ConstPtr model):
        OpenSotTaskAdapter(task, model)
    {
        _ci_angmom = std::dynamic_pointer_cast<AngularMomentum>(task);

        if(!_ci_angmom) throw std::runtime_error("Provided task description "
                                                 "does not have expected type 'AngularMomentum'");

    }

    TaskPtr OpenSotAngularMomentum::constructTask()
    {
        Eigen::VectorXd q;
        _model->getJointPosition(q);

        _sot_angmom = boost::make_shared<AngularMomentumSoT>(q,
                                                             const_cast<ModelInterface&>(*_model));

        return _sot_angmom;
    }

    bool OpenSotAngularMomentum::initialize()
    {
        return OpenSotTaskAdapter::initialize();
    }

    void OpenSotAngularMomentum::update(double time, double period)
    {
        OpenSotTaskAdapter::update(time, period);

        _sot_angmom->setReference(_ci_angmom->getReference() * period);
    }

    CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotAngularMomentum)



