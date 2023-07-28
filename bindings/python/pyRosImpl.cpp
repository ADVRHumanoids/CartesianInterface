#include <pyRosImpl.h>


#include "ros/client_api/CartesianRos.h"
#include "ros/client_api/PosturalRos.h"
#include "ros/client_api/InteractionRos.h"


#include "problem/Cartesian.h"
#include "problem/Postural.h"
#include "problem/Interaction.h"
#include "problem/Com.h"

#include <cartesian_interface/utils/RobotStatePublisher.h>

PYBIND11_MODULE(pyci, m) {

    /* Create binding for ControlType enum */
    py::enum_<ControlType>(m, "ControlType", py::arithmetic())
            .value("Position", ControlType::Position)
            .value("Velocity", ControlType::Velocity);

    py::enum_<State>(m, "State", py::arithmetic())
            .value("Online", State::Online)
            .value("Reaching", State::Reaching);

    py::enum_<ActivationState>(m, "ActivationState", py::arithmetic())
            .value("Enabled", ActivationState::Enabled)
            .value("Disabled", ActivationState::Disabled);

    py::class_<Trajectory::WayPoint>(m, "WayPoint")
            .def(py::init())
            .def(py::init<Eigen::Affine3d, double>())
            .def_readwrite("frame", &Trajectory::WayPoint::frame)
            .def_readwrite("time", &Trajectory::WayPoint::time)
            .def("__repr__", waypoint_repr);

    py::class_<ProblemDescription>(m, "ProblemDescription")
            .def("getNumTasks", &ProblemDescription::getNumTasks)
            .def("getTask", pb_get_task_id)
            .def("getTaskByName", pb_get_task_name)
            .def("getBounds", &ProblemDescription::getBounds)
            ;

    py::class_<RosServerClass>(m, "RosServerClass")
            .def(py::init(&make_ros_server_class),
                 py::arg("ci"),
                 py::arg("ros_namespace") = "cartesian",
                 py::arg("tf_prefix") = "ci",
                 py::arg("publish_tf") = true)
            .def("run", &RosServerClass::run)
            ;


    py::class_<TaskDescription,
            TaskDescription::Ptr>(m, "Task")
            .def("getName", &TaskDescription::getName)
            .def("getType", &TaskDescription::getType)
            .def("getLambda", &TaskDescription::getLambda)
            .def("getLambda2", &TaskDescription::getLambda2)
            .def("getWeight", &TaskDescription::getWeight)
            .def("getSize", &TaskDescription::getSize)
            .def("getIndices", &TaskDescription::getIndices)
            .def("getActivationState", &TaskDescription::getActivationState)
            .def("setLambda", &TaskDescription::setLambda)
            .def("setLambda2", &TaskDescription::setLambda2)
            .def("setWeight", &TaskDescription::setWeight)
            .def("setActivationState", &TaskDescription::setActivationState)
            .def("disable", [](TaskDescription& t){ t.setActivationState(ActivationState::Disabled); })
            .def("enable", [](TaskDescription& t){ t.setActivationState(ActivationState::Enabled); });

    py::class_<CartesianTask,
            TaskDescription,
            CartesianTask::Ptr>(m, "CartesianTask", py::multiple_inheritance())
            .def("getBaseLink", &CartesianTask::getBaseLink, py::return_value_policy::reference_internal)
            .def("getTaskState", &CartesianTask::getTaskState)
            .def("getDistalLink", &CartesianTask::getDistalLink, py::return_value_policy::reference_internal)
            .def("getControlMode", &CartesianTask::getControlMode)
            .def("getCurrentSegmentId", &CartesianTask::getCurrentSegmentId)
            .def("getVelocityLimits", py_task_get_vel_lims)
            .def("getAccelerationLimits", py_task_get_acc_lims)
            .def("setBaseLink", &CartesianTask::setBaseLink)
            .def("setWayPoints", &CartesianTask::setWayPoints)
            .def("setPoseTarget", &CartesianTask::setPoseTarget)
            .def("setVelocityReference", &CartesianTask::setVelocityReference)
            .def("setControlMode", &CartesianTask::setControlMode)
            .def("setPoseReference", &CartesianTask::setPoseReference)
            .def("setVelocityLimits", &CartesianTask::setVelocityLimits)
            .def("setAccelerationLimits", &CartesianTask::setAccelerationLimits)
            .def("getPoseReference", py_task_get_pose_reference)
            .def("abort", &CartesianTask::abort);

    py::class_<ComTask,
            CartesianTask,
            ComTask::Ptr>(m, "ComTask", py::multiple_inheritance());

    py::class_<PosturalTask,
            TaskDescription,
            PosturalTask::Ptr>(m, "PosturalTask", py::multiple_inheritance())
            .def("setReferencePosture", (void (PosturalTask::*)(const XBot::JointNameMap&)) &PosturalTask::setReferencePosture)
            .def("getReferencePostureMap",  py_postural_get_reference_map);

    py::class_<InteractionTask,
            CartesianTask,
            InteractionTask::Ptr>(m, "InteractionTask", py::multiple_inheritance())
            .def("getImpedance", &InteractionTask::getImpedance)
            .def("getForceReference", &InteractionTask::getForceReference)
            .def("getForceLimits", py_task_get_force_lims)
            .def("setImpedance", &InteractionTask::setImpedance)
            .def("setForceReference", &InteractionTask::setForceReference)
            .def("setForceLimits", &InteractionTask::setForceLimits)
            .def("setStiffnessTransition", &InteractionTask::setStiffnessTransition)
            .def("abortStiffnessTransition", &InteractionTask::abortStiffnessTransition)
            .def("getStiffnessState", &InteractionTask::getStiffnessState);

    py::class_<ClientApi::InteractionRos,
            InteractionTask,
            ClientApi::InteractionRos::Ptr>(m, "InteractionRos", py::multiple_inheritance())
            .def("waitReachCompleted", py_wait_reach_completed_gil_release_inte);

    py::class_<InteractionTaskImpl,
            InteractionTask,
            InteractionTaskImpl::Ptr>(m, "InteractionTaskImpl", py::multiple_inheritance());

    py::class_<ClientApi::CartesianRos,
            CartesianTask,
            ClientApi::CartesianRos::Ptr>(m, "CartesianTaskRos", py::multiple_inheritance())
            .def("waitReachCompleted", py_wait_reach_completed_gil_release_cart);

    py::class_<ClientApi::PosturalRos,
            PosturalTask,
            ClientApi::PosturalRos::Ptr>(m, "PosturalRos", py::multiple_inheritance())

    py::class_<CartesianTaskImpl,
            CartesianTask,
            CartesianTaskImpl::Ptr>(m, "CartesianTaskImpl", py::multiple_inheritance());

    py::class_<ComTaskImpl,
            CartesianTaskImpl,
            ComTaskImpl::Ptr>(m, "ComTaskImpl", py::multiple_inheritance());

    py::class_<PosturalTaskImpl,
            PosturalTask,
            PosturalTaskImpl::Ptr>(m, "PosturalTaskImpl", py::multiple_inheritance());

    py::class_<CartesianInterfaceImpl,
            CartesianInterfaceImpl::Ptr>(m, "CartesianInterface")
            .def_static("MakeInstance", make_ci,
                    py::arg("solver"),
                    py::arg("problem"),
                    py::arg("model"),
                    py::arg("dt"),
                    py::arg("log_path") = "/tmp")
            .def("getSolution", &CartesianInterfaceImpl::getSolution)
            .def("getTask", &CartesianInterfaceImpl::getTask, py::return_value_policy::reference_internal)
            .def("update", &CartesianInterfaceImpl::update)
            .def("getTaskList", &CartesianInterfaceImpl::getTaskList)
            .def("reset", (bool (CartesianInterfaceImpl::*)(double)) &CartesianInterfaceImpl::reset);

    py::class_<RosClient,
            CartesianInterfaceImpl,
            RosClient::Ptr>(m, "CartesianInterfaceRos")
            .def(py::init<std::string>(), py::arg("namespace") = "cartesian")
            .def("__repr__", ci_repr)
            .def("update", &RosClient::update, py::arg("time") = 0, py::arg("period") = 0)
            .def("getControlMode", &RosClient::getControlMode)
            .def("setControlMode", &RosClient::setControlMode)
            .def("getBaseLink", &RosClient::getBaseLink)
            .def("setBaseLink", &RosClient::setBaseLink)
            .def("loadController", &RosClient::loadController,
                 py::arg("controller_name"),
                 py::arg("problem_description_name") = "",
                 py::arg("problem_description_string") = "",
                 py::arg("force_reload") = true)
            .def("getVelocityLimits", py_get_velocity_limits)
            .def("getAccelerationLimits", py_get_acceleration_limits)
            .def("setVelocityLimits", &RosClient::setVelocityLimits)
            .def("setAccelerationLimits", &RosClient::setAccelerationLimits)
            .def("setReferencePosture",  (bool (RosClient::*)(const XBot::JointNameMap&)) &RosClient::setReferencePosture)
            .def("setTargetPose", py_send_target_pose,
                 py::arg("task_name"),
                 py::arg("pose"),
                 py::arg("time"),
                 py::arg("incremental") = false)
            .def("setWaypoints", py_send_waypoints,
                 py::arg("task_name"),
                 py::arg("waypoints"),
                 py::arg("incremental") = false)
            .def("waitReachCompleted", &RosClient::waitReachCompleted,
                 py::arg("task_name"),
                 py::arg("timeout") = 0.0)
            .def("getPoseReference", py_get_pose_reference)
            .def("getDesiredInteraction", py_get_interaction_reference)
            .def("setPoseReference", &RosClient::setPoseReference)
            .def("setForceReference", &RosClient::setForceReference)
            .def("setDesiredStiffness", &RosClient::setDesiredStiffness)
            .def("setDesiredDamping", &RosClient::setDesiredDamping)
            .def("setStiffnessTransition", py_send_stiffness_waypoints,
                 py::arg("task_name"),
                 py::arg("translational_stiffness"),
                 py::arg("rotational_stiffness"),
                 py::arg("time"))
            .def("waitStiffnessTransitionCompleted", &RosClient::waitStiffnessTransitionCompleted,
                 py::arg("task_name"),
                 py::arg("timeout") = 0.0)
            .def("abortStiffnessTransition", &RosClient::abortStiffnessTransition,
                 py::arg("task_name"))
            .def("resetWorld", (bool (RosClient::*)(const Eigen::Affine3d&)) &RosClient::resetWorld)
            .def("resetWorld", (bool (RosClient::*)(const std::string&))     &RosClient::resetWorld)
            .def("setVelocityReference", (bool (RosClient::*)(const std::string&,
                                                              const Eigen::Vector6d&))  &RosClient::setVelocityReference)
            //        .def("setVelocityReferenceAsync", &RosClient::setVelocityReferenceAsync)
            .def("setVelocityReference", (bool (RosClient::*)(const std::string&,
                                                              const Eigen::Vector6d&,
                                                              const std::string&))  &RosClient::setVelocityReference)
            //        .def("stopVelocityReferenceAsync", &RosClient::stopVelocityReferenceAsync)
            .def("getPoseFromTf", py_get_pose_from_tf);


    /* Robot state pub util */
    py::class_<Utils::RobotStatePublisher>(m, "RobotStatePublisher")
        .def(py::init<XBot::ModelInterface::ConstPtr>())
        .def("publishTransforms",
             [](Utils::RobotStatePublisher& r, std::string pref)
             {
                 r.publishTransforms(ros::Time::now(), pref);
             })
        ;


}



