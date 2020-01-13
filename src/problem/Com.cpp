#include <cartesian_interface/problem/Com.h>

using namespace XBot::Cartesian;

ComTask::ComTask():
    CartesianTask("com", "com", "world")
{

}

XBot::Cartesian::ComTask::ComTask(YAML::Node node,
                                  XBot::ModelInterface::ConstPtr model):
    CartesianTask(node, model)
{

}

