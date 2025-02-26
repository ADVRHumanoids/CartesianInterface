#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/ros/RosClient.h>
#include <cartesian_interface/problem/Interaction.h>
#include "ros/client_api/CartesianRos.h"
#include "ros/client_api/InteractionRos.h"

namespace py = pybind11;
using namespace XBot::Cartesian;

CartesianInterfaceImpl::Ptr make_ci(std::string solver_name,
                                    std::string problem,
                                    XBot::ModelInterface::Ptr model,
                                    double dt,
                                    std::string log_path)
{
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(dt, log_path),
                model
            );

    auto ik_pb_yaml = YAML::Load(problem);

    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    if(solver_name.empty())
    {
        return std::make_shared<CartesianInterfaceImpl>(ik_pb, ctx);
    }

    return CartesianInterfaceImpl::MakeInstance(solver_name, ik_pb, ctx);

}

RosServerClass::UniquePtr make_ros_server_class(
        CartesianInterfaceImpl::Ptr ci,
        std::string ros_namespace,
        std::string tf_prefix,
        bool publish_tf)
{
    RosServerClass::Options opt;
    opt.ros_namespace = ros_namespace;
    opt.tf_prefix = tf_prefix;
    opt.publish_tf = publish_tf;

    return std::make_unique<RosServerClass>(ci, opt);
}

auto pb_get_task_id(const ProblemDescription& self,
                 int id)
{
    return self.getTask(id);
}

auto pb_get_task_name(ProblemDescription& self,
                      std::string name)
{
    return self.getTask(name);
}


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

auto py_postural_get_reference_map(const PosturalTask& p)
{
    XBot::JointNameMap m;
    p.getReferencePosture(m);
    return m;
}

auto py_postural_get_reference(const PosturalTask& p)
{
    Eigen::VectorXd q;
    p.getReferencePosture(q);
    return q;
}

auto py_task_get_vel_lims(const CartesianTask& t)
{
    double l, a;
    t.getVelocityLimits(l, a);
    return std::make_tuple(l, a);
}

auto py_task_get_acc_lims(const CartesianTask& t)
{
    double l, a;
    t.getAccelerationLimits(l, a);
    return std::make_tuple(l, a);
}

auto py_task_get_force_lims(const InteractionTask& t)
{
    Eigen::Vector6d l;
    t.getForceLimits(l);
    return l;
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

auto py_send_stiffness_waypoints(RosClient& r,
                       const std::string& ee,
                       const std::vector<Eigen::Vector3d> kt,
                       const std::vector<Eigen::Vector3d> kr,
                       const std::vector<double> t)
{
    if (kt.size() != kr.size() || kt.size() != t.size() || kt.size() < 1)
    {
        return false;
    }

    Interpolator<Eigen::Matrix6d>::WayPointVector wpv;

    for (int i = 0; i < t.size(); i++)
    {
        Interpolator<Eigen::Matrix6d>::WayPoint wp;
        wp.value.setZero();

        wp.value.diagonal().head(3) = kt[i];
        wp.value.diagonal().tail(3) = kr[i];
        wp.time = t[i];

        wpv.push_back(wp);
    }

    return r.setStiffnessTransition(ee, wpv);
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




auto py_wait_reach_completed_gil_release_cart(ClientApi::CartesianRos& c, double timeout)
{
    py::gil_scoped_release gil_release;
    return c.waitReachCompleted(timeout);
}

auto py_wait_reach_completed_gil_release_inte(ClientApi::InteractionRos& c, double timeout)
{
    return py_wait_reach_completed_gil_release_cart(c, timeout);
}
