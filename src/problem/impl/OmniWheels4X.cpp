#include "problem/OmniWheels4X.h"

using namespace XBot::Cartesian;

OmniWheels4X::OmniWheels4X(YAML::Node yaml, Context::ConstPtr context):
    TaskDescriptionImpl(yaml,
                        context,
                        "OmniWheels4X",
                        context->model()->getJointNum()),
    _base_link("base_link")
{
    if(yaml["l1"])
    {
        _l1 = yaml["l1"].as<double>();
    }
    else
        throw std::invalid_argument("Missing mandatory l1 argument!");

    if(yaml["l2"])
    {
        _l2 = yaml["l2"].as<double>();
    }
    else
        throw std::invalid_argument("Missing mandatory l2 argument!");

    if(yaml["wheel_radius"])
    {
        _r = yaml["wheel_radius"].as<double>();
    }
    else
        throw std::invalid_argument("Missing mandatory wheel_radius argument!");

    if(yaml["base_link"])
    {
        _base_link = yaml["base_link"].as<std::string>();
    }

    if(yaml["joint_wheels_name"])
    {
        for(auto joint_name : yaml["joint_wheels_name"])
            _joint_wheels_name.push_back(joint_name.as<std::string>());
    }
    else
        throw std::invalid_argument("Missing mandatory joint_wheels_name argument!");


}
