#include "CartesianRos.h"

using namespace XBot::Cartesian;

CartesianRos::CartesianRos(std::string name):
    TaskRos(name)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}


void CartesianRos::getVelocityLimits(double & max_vel_lin, double & max_vel_ang) const
{
}

void CartesianRos::getAccelerationLimits(double & max_acc_lin, double & max_acc_ang) const
{
}

void CartesianRos::setVelocityLimits(double max_vel_lin, double max_vel_ang)
{
}

void CartesianRos::setAccelerationLimits(double max_acc_lin, double max_acc_ang)
{
}

void CartesianRos::enableOtg()
{
}

State CartesianRos::getTaskState() const
{
}

const std::string & CartesianRos::getBaseLink() const
{
}

bool CartesianRos::setBaseLink(const std::string & new_base_link)
{
}

const std::string & CartesianRos::getDistalLink() const
{
}

bool CartesianRos::getCurrentPose(Eigen::Affine3d & base_T_ee) const
{
}

bool CartesianRos::getPoseReference(Eigen::Affine3d & base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const
{
}

bool CartesianRos::getPoseReferenceRaw(Eigen::Affine3d & base_T_ref, Eigen::Vector6d * base_vel_ref, Eigen::Vector6d * base_acc_ref) const
{
}

bool CartesianRos::setPoseReference(const Eigen::Affine3d & base_T_ref)
{
}

bool CartesianRos::setPoseReferenceRaw(const Eigen::Affine3d & base_T_ref)
{
}

bool CartesianRos::setVelocityReference(const Eigen::Vector6d & base_vel_ref)
{
}

bool CartesianRos::getPoseTarget(Eigen::Affine3d & base_T_ref) const
{
}

bool CartesianRos::setPoseTarget(const Eigen::Affine3d & base_T_ref, double time)
{
}

int CartesianRos::getCurrentSegmentId() const
{
}

bool CartesianRos::setWayPoints(const Trajectory::WayPointVector & way_points)
{
}

void CartesianRos::abort()
{
}

CartesianRos::~CartesianRos() {}

Eigen::Affine3d CartesianRos::get_current_pose() const
{
}
