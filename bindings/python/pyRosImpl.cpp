#include <pyRosImpl.h>


PYBIND11_MODULE(pyci, m) {
    
    /* Create binding for ControlType enum */
    py::enum_<ControlType>(m, "ControlType", py::arithmetic())
        .value("Position", ControlType::Position)
        .value("Velocity", ControlType::Velocity);

    py::enum_<ActivationState>(m, "ActivationState", py::arithmetic())
        .value("Enabled", ActivationState::Enabled)
        .value("Disabled", ActivationState::Disabled);

    py::class_<Trajectory::WayPoint>(m, "WayPoint")
        .def(py::init())
        .def(py::init<Eigen::Affine3d, double>())
        .def_readwrite("frame", &Trajectory::WayPoint::frame)
        .def_readwrite("time", &Trajectory::WayPoint::time)
        .def("__repr__", waypoint_repr);
    
    py::class_<RosClient>(m, "CartesianInterfaceRos")
        .def(py::init<std::string>(), py::arg("namespace") = "cartesian")
        .def("__repr__", ci_repr)
        .def("update", &RosClient::update, py::arg("time") = 0, py::arg("period") = 0)
        .def("getTaskList", &RosClient::getTaskList)
        .def("reset", (bool (RosClient::*)(void)) &RosClient::reset)
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
        .def("setReferencePosture", &RosClient::setReferencePosture)
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
        
    
}



