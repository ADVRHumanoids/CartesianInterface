#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <cartesian_interface/ros/RosImpl.h>

namespace py = pybind11;
using namespace XBot::Cartesian;


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
    T.linear() = Eigen::Quaterniond(qcoeff).toRotationMatrix();
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
             