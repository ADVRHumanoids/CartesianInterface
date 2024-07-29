#include "problem/Gaze.h"

using namespace XBot::Cartesian;

GazeTaskImpl::GazeTaskImpl(Context::ConstPtr context,
                  std::string name,
                  std::string distal_link,
                  std::string base_link):
    CartesianTaskImpl(context, name, distal_link, base_link)
{

}

GazeTaskImpl::GazeTaskImpl(YAML::Node node,
                         Context::ConstPtr context):
    CartesianTaskImpl(node, context)
{

}
