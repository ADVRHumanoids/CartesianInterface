#include "problem/Com.h"

using namespace XBot::Cartesian;

ComTaskImpl::ComTaskImpl(Context::ConstPtr context):
    CartesianTaskImpl(context, "com", "com", "world")
{

}

ComTaskImpl::ComTaskImpl(YAML::Node node,
                         Context::ConstPtr context):
    CartesianTaskImpl(node, context)
{

}

