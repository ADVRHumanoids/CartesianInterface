#include <pyRosImpl.h>


PYBIND11_MODULE(pyci_ros, m) {
    
    py::enum_<CartesianInterface::ControlType>(m, "ControlType", py::arithmetic())
        .value("Position", CartesianInterface::ControlType::Position)
        .value("Velocity", CartesianInterface::ControlType::Velocity)
        .value("Disabled", CartesianInterface::ControlType::Disabled);
        
    py::class_<WayPoint>(m, "WayPoint")
        .def(py::init())
        .def_readwrite("position", &WayPoint::position)
        .def_readwrite("orientation", &WayPoint::orientation)
        .def_readwrite("time", &WayPoint::time)
        .def("__repr__", &WayPoint::repr);
    
    py::class_<RosImpl>(m, "CartesianInterfaceRos")
        .def(py::init())
        .def("update", &RosImpl::update, py::arg("time") = 0, py::arg("period") = 0)
        .def("getTaskList", &RosImpl::getTaskList)
        .def("getControlMode", &RosImpl::getControlMode)
        .def("setControlMode", &RosImpl::setControlMode)
        .def("getBaseLink", &RosImpl::getBaseLink)
        .def("setBaseLink", &RosImpl::setBaseLink)
        .def("setTargetPose", py_send_target_pose, 
             py::arg("task_name"), 
             py::arg("pos"), 
             py::arg("rot"), 
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
        .def("setPoseReference", py_set_pose_reference)
        .def("setPoseReference", py_set_pose_reference_novel);
        
    
}



