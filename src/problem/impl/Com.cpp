#include "Com.h"

using namespace XBot::Cartesian;

ComTaskImpl::ComTaskImpl(ModelInterface::ConstPtr model):
    CartesianTaskImpl(model, "com", "com", "world")
{

}

ComTaskImpl::ComTaskImpl(YAML::Node node,
                         XBot::ModelInterface::ConstPtr model):
    CartesianTaskImpl(node, model)
{

}

