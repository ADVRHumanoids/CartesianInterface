#include "TaskRos.h"

using namespace XBot::Cartesian;

TaskRos::TaskRos(std::string name)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}

bool TaskRos::validate()
{
}

void TaskRos::update(double time, double period)
{
}

void TaskRos::reset()
{
}

void TaskRos::log(XBot::MatLogger::Ptr logger, bool init_logger, int buf_size)
{
}


const Eigen::MatrixXd & TaskRos::getWeight() const
{
}

bool TaskRos::setWeight(const Eigen::MatrixXd & value)
{
}

const std::vector<int> & TaskRos::getIndices() const
{
}

void TaskRos::setIndices(const std::vector<int> & value)
{
}

double TaskRos::getLambda() const
{
}

void TaskRos::setLambda(double value)
{
}

const std::vector<std::string> & TaskRos::getDisabledJoints() const
{
}

void TaskRos::setDisabledJoints(const std::vector<std::string> & value)
{
}

ActivationState TaskRos::getActivationState() const
{
}

bool TaskRos::setActivationState(const ActivationState & value)
{
}

TaskRos::~TaskRos() {}
