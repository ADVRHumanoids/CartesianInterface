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

std::string ci_repr(const RosClient& r)
{
    std::stringstream ss;
    ss << r;
    return ss.str();
}

auto py_task_get_pose_reference(const CartesianTask& r)
{
    Eigen::Affine3d T;
    Eigen::Vector6d v, a;

    r.getPoseReference(T, &v, &a);

    return std::make_tuple(T, v, a);

}


auto py_get_pose_reference(const RosClient& r, const std::string& ee)
{
    Eigen::Affine3d T;
    Eigen::Vector6d v, a;
    
    r.getPoseReference(ee, T, &v, &a);
    
    return std::make_tuple(T, v, a);
    
}

auto py_get_interaction_reference(const RosClient& r, const std::string& ee)
{
    Eigen::Matrix6d k, d;
    Eigen::Vector6d f;
    
    r.getDesiredInteraction(ee, f, k, d);
    
    return std::make_tuple(f, k, d);
    
}

auto py_send_waypoints(RosClient& r,
                       const std::string& ee, 
                       const std::vector<Trajectory::WayPoint>& wpv,
                       bool incremental = false
                      )
{
    return r.setWayPoints(ee, wpv, incremental);
}
    
auto py_send_target_pose(RosClient& r,
                         const std::string& ee,
                         const Eigen::Affine3d& T, 
                         double time,
                         bool incremental
                        )
{
    Trajectory::WayPoint wp;
    wp.time = time;
    wp.frame = T;
    return r.setWayPoints(ee, {wp}, incremental);
}

auto py_get_velocity_limits(RosClient& r, const std::string& ee)
{
    double lin = 0.0, ang = 0.0;
    r.getVelocityLimits(ee, lin, ang);
    return std::make_tuple(lin, ang);
}

auto py_get_acceleration_limits(RosClient& r, const std::string& ee)
{
    double lin = 0.0, ang = 0.0;
    r.getAccelerationLimits(ee, lin, ang);
    return std::make_tuple(lin, ang);
}

auto py_get_pose_from_tf(RosClient& r, const std::string& source_frame, const std::string& target_frame)
{
    Eigen::Affine3d T;

    if(!r.getPoseFromTf(source_frame, target_frame, T))
    {
        throw std::runtime_error("Unable to get pose from '" + source_frame  + "'' to '" + target_frame + "'");
    }


    return T;

}
