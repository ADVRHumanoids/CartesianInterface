#include <pyRosImpl.h>


PYBIND11_MODULE(pyci, m) {
    
    /* Create binding for ControlType enum */
    py::enum_<ControlType>(m, "ControlType", py::arithmetic())
        .value("Position", ControlType::Position)
        .value("Velocity", ControlType::Velocity)
        .value("Disabled", ControlType::Disabled);
        
    py::class_<Trajectory::WayPoint>(m, "WayPoint")
        .def(py::init())
        .def(py::init<Eigen::Affine3d, double>())
        .def_readwrite("frame", &Trajectory::WayPoint::frame)
        .def_readwrite("time", &Trajectory::WayPoint::time)
        .def("__repr__", waypoint_repr);
    
    py::class_<RosImpl>(m, "CartesianInterfaceRos")
        .def(py::init<std::string>(), py::arg("namespace") = "")
        .def("__repr__", ci_repr)
        .def("update", &RosImpl::update, py::arg("time") = 0, py::arg("period") = 0)
        .def("getTaskList", &RosImpl::getTaskList)
        .def("getControlMode", &RosImpl::getControlMode)
        .def("setControlMode", &RosImpl::setControlMode)
        .def("getBaseLink", &RosImpl::getBaseLink)
        .def("setBaseLink", &RosImpl::setBaseLink)
        .def("loadController", &RosImpl::loadController)
        .def("getVelocityLimits", py_get_velocity_limits)
        .def("getAccelerationLimits", py_get_acceleration_limits)
        .def("setVelocityLimits", &RosImpl::setVelocityLimits)
        .def("setAccelerationLimits", &RosImpl::setAccelerationLimits)
        .def("setReferencePosture", &RosImpl::setReferencePosture)
        .def("setTargetPose", py_send_target_pose, 
             py::arg("task_name"), 
             py::arg("pose"), 
             py::arg("time"), 
             py::arg("incremental") = false)
        .def("setWaypoints", py_send_waypoints, 
             py::arg("task_name"), 
             py::arg("waypoints"), 
             py::arg("incremental") = false)
        .def("waitReachCompleted", &RosImpl::waitReachCompleted, 
             py::arg("task_name"), 
             py::arg("timeout") = 0.0)
        .def("getPoseReference", py_get_pose_reference)
        .def("setPoseReference", &RosImpl::setPoseReference)
        .def("setVelocityReference", &RosImpl::setVelocityReference)
        .def("setVelocityReferenceAsync", &RosImpl::setVelocityReferenceAsync)
        .def("stopVelocityReferenceAsync", &RosImpl::stopVelocityReferenceAsync);
        
    
}



