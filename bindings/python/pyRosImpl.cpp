#include <pyRosImpl.h>


PYBIND11_MODULE(pyci_ros, m) {
    
    py::enum_<CartesianInterface::ControlType>(m, "ControlType", py::arithmetic())
        .value("Position", CartesianInterface::ControlType::Position)
        .value("Velocity", CartesianInterface::ControlType::Velocity)
        .value("Disabled", CartesianInterface::ControlType::Disabled);
    
    py::class_<RosImpl>(m, "CartesianInterfaceRos")
        .def(py::init())
        .def("update", &RosImpl::update)
        .def("getControlMode", &RosImpl::getControlMode)
        .def("getBaseLink", &RosImpl::getBaseLink)
        .def("getPoseReference", py_get_pose_reference)
        .def("setPoseReference", py_set_pose_reference)
        .def("setPoseReference", py_set_pose_reference_novel);
        
}



