#include "AngularMomentum.h"

#include "fmt/format.h"

using namespace XBot::Cartesian;


AngularMomentumImpl::AngularMomentumImpl(YAML::Node task_node,
                                         XBot::ModelInterface::ConstPtr model):
    TaskDescriptionImpl(task_node,
                        model,
                        "AngularMomentum",
                        3),
    _lref(0,0,0),
    _ref_timeout(-1)
{
    /* Here you can parse custom YAML fields from task_node */

}

Eigen::Vector3d AngularMomentumImpl::getReference() const
{
    return _lref;
}

void AngularMomentumImpl::setReference(const Eigen::Vector3d& lref)
{
    _lref = lref;
    _ref_timeout = getTime() + REF_TTL;
}



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
        logger->createVectorVariable(getName() + "_ref", 3, 1, buf_size);
        return;
    }

    logger->add(getName() + "_ref", _lref);
}


CARTESIO_REGISTER_TASK_PLUGIN(AngularMomentumImpl)
