#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <cartesian_interface/ros/RosImpl.h>

namespace py = pybind11;
using namespace XBot::Cartesian;

struct WayPoint 
{
    Eigen::Vector3d position;
    Eigen::Vector4d orientation;
    double time;
    
    WayPoint(): position(0,0,0), orientation(0,0,0,1), time(0)
    {
    }
    
    std::string repr()
    {
        std::stringstream ss;
        ss << "pyci_ros.WayPoint with pos: (" << 
            position.transpose().format(Eigen::IOFormat(2)) << "), rot: (" << 
            orientation.transpose().format(Eigen::IOFormat(2)) << "), time: " << time;
        return ss.str();
    }
           
        
    
};

auto py_get_pose_reference(const RosImpl& r, const std::string& ee)
{
    Eigen::Affine3d T;
    Eigen::Vector6d v, a;
    
    r.getPoseReference(ee, T, &v, &a);
    
    Eigen::Quaterniond q(T.linear());
    Eigen::Vector3d t = T.translation();
    Eigen::Vector4d qc = q.coeffs();
    
    return std::make_tuple(t, qc, v, a);
    
}

bool py_set_pose_reference(RosImpl& r, 
                           const std::string& ee,
                           const Eigen::Vector3d& t,
                           const Eigen::Vector4d& qcoeff,
                           const Eigen::Vector6d& vel_ref
                          )
{
    Eigen::Affine3d T;
    T.setIdentity();
    T.linear() = Eigen::Quaterniond(qcoeff).normalized().toRotationMatrix();
    T.translation() = t;
    
    Eigen::Vector6d a;
    a.setZero();
    
    return r.setPoseReference(ee, T, vel_ref, a);
    
}

bool py_set_pose_reference_novel(RosImpl& r, 
                           const std::string& ee,
                           const Eigen::Vector3d& t,
                           const Eigen::Vector4d& qcoeff
                          )
{
    return py_set_pose_reference(r, ee, t, qcoeff, Eigen::Vector6d::Zero());
}

auto py_send_waypoints(RosImpl& r, 
                       const std::string& ee, 
                       const std::vector<WayPoint>& arg_wpv,
                       bool incremental = false
                      )
{
    Trajectory::WayPointVector wpv;
    for(auto wp : arg_wpv)
    {
        Eigen::Affine3d T;
        T.setIdentity();
        T.linear() = Eigen::Quaterniond(wp.orientation).normalized().toRotationMatrix();
        T.translation() = wp.position;
        
        wpv.emplace_back(T, wp.time);
    }
    
    return r.setWayPoints(ee, wpv, incremental);
}
    
auto py_send_target_pose(RosImpl& r, 
                         const std::string& ee,
                         const Eigen::Vector3d& t,
                         const Eigen::Vector4d& qcoeff, 
                         double time,
                         bool incremental
                        )
{
    Eigen::Affine3d T;
    T.setIdentity();
    T.linear() = Eigen::Quaterniond(qcoeff).normalized().toRotationMatrix();
    T.translation() = t;
    
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