#include "Manipulability.h"

#include "fmt/format.h"

using namespace XBot::Cartesian;

std::string get_distal_link(YAML::Node task_node)
{
    if(auto n = task_node["distal_link"])
    {
        return n.as<std::string>();
    }
    else
        throw std::runtime_error("distal_link mandatory in Manipulability task!");
}


ManipulabilityImpl::ManipulabilityImpl(YAML::Node task_node,
                                     Context::ConstPtr context):
    TaskDescriptionImpl(task_node,
                        context,
                        "Manipulability_" + get_distal_link(task_node),
                        context->model()->getNv()),
    _step_size(1e-3)
{
    /* Here you can parse custom YAML fields from task_node */

    if(auto n = task_node["step_size"])
    {
        _step_size = n.as<double>();
    }

    _distal_link = get_distal_link(task_node);
    if(_distal_link == "com")
        _base_link = "world";
    else
    {
        if(auto n = task_node["base_link"])
        {
            _base_link = n.as<std::string>();
        }
        else
            _base_link = "world";
    }
}

const std::string& ManipulabilityImpl::getBaseLink() const
{
    return _base_link;
}

const std::string& ManipulabilityImpl::getDistalLink() const
{
    return _distal_link;
}

double ManipulabilityImpl::getStepSize() const
{
    return _step_size;
}

void XBot::Cartesian::ManipulabilityImpl::update(double time, double period)
{
    // call base class
    TaskDescriptionImpl::update(time, period);

    // any custom update action that the task may need
}

void XBot::Cartesian::ManipulabilityImpl::reset()
{
    // call base class
    TaskDescriptionImpl::reset();

    // any custom reset action that the task may need
}

void XBot::Cartesian::ManipulabilityImpl::log(MatLogger2::Ptr logger,
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


CARTESIO_REGISTER_TASK_PLUGIN(ManipulabilityImpl, Manipulability)
