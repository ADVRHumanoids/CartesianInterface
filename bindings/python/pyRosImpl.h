#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <cartesian_interface/ros/RosImpl.h>

namespace py = pybind11;
using namespace XBot::Cartesian;

std::string waypoint_repr(const Trajectory::WayPoint& w)
{
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    
    std::stringstream ss;
    ss <<   "translation: " << w.frame.translation().transpose().format(CleanFmt);
    ss << "\nrotation   : " << Eigen::Quaterniond(w.frame.linear()).coeffs().transpose().format(CleanFmt);
    ss << "\ntime       : " << w.time;
    return ss.str();
}

auto py_get_pose_reference(const RosImpl& r, const std::string& ee)
{
    Eigen::Affine3d T;
    Eigen::Vector6d v, a;
    
    r.getPoseReference(ee, T, &v, &a);
    
    return std::make_tuple(T, v, a);
    
}

bool py_set_pose_reference(RosImpl& r, 
                           const std::string& ee,
                           const Eigen::Affine3d& Tref,
                           const Eigen::Vector6d& vel_ref
                          )
{
    Eigen::Vector6d a;
    a.setZero();
    
    return r.setPoseReference(ee, Tref, vel_ref, a);
    
}

bool py_set_pose_reference_novel(RosImpl& r, 
                           const std::string& ee,
                           const Eigen::Affine3d& Tref
                          )
{
    return py_set_pose_reference(r, ee, Tref, Eigen::Vector6d::Zero());
}

auto py_send_waypoints(RosImpl& r, 
                       const std::string& ee, 
                       const std::vector<Trajectory::WayPoint>& wpv,
                       bool incremental = false
                      )
{
    return r.setWayPoints(ee, wpv, incremental);
}
    
auto py_send_target_pose(RosImpl& r, 
                         const std::string& ee,
                         const Eigen::Affine3d& T, 
                         double time,
                         bool incremental
                        )
{
    return r.setTargetPose(ee, T, time, incremental);
}

auto py_get_velocity_limits(RosImpl& r, const std::string& ee)
{
    double lin = 0.0, ang = 0.0;
    r.getVelocityLimits(ee, lin, ang);
    return std::make_tuple(lin, ang);
}

auto py_get_acceleration_limits(RosImpl& r, const std::string& ee)
{
    double lin = 0.0, ang = 0.0;
    r.getAccelerationLimits(ee, lin, ang);
    return std::make_tuple(lin, ang);
}