#include "MinEffort.h"

#include "fmt/format.h"

using namespace XBot::Cartesian;


MinimumEffortImpl::MinimumEffortImpl(YAML::Node task_node,
                                     Context::ConstPtr context):
    TaskDescriptionImpl(task_node,
                        context,
                        "MinimumEffort",
                        context->model()->getNv()),
    _step_size(1e-3)
{
    /* Here you can parse custom YAML fields from task_node */

    if(auto n = task_node["step_size"])
    {
        _step_size = n.as<double>();
    }
}

double MinimumEffortImpl::getStepSize() const
{
    return _step_size;
}

void XBot::Cartesian::MinimumEffortImpl::update(double time, double period)
{
    // call base class
    TaskDescriptionImpl::update(time, period);

    // any custom update action that the task may need
}

void XBot::Cartesian::MinimumEffortImpl::reset()
{
    // call base class
    TaskDescriptionImpl::reset();

    // any custom reset action that the task may need
}

void XBot::Cartesian::MinimumEffortImpl::log(MatLogger2::Ptr logger,
                                               bool init_logger,
                                               int buf_size)
{
    // call base class
    TaskDescriptionImpl::log(logger, init_logger, buf_size);

    if(init_logger)
    {
        // call logger->create
        return;
    }

    // call logger->add
}


CARTESIO_REGISTER_TASK_PLUGIN(MinimumEffortImpl, MinimumEffort)
