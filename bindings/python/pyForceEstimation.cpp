#include <pyRosImpl.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <xbot2_interface/xbotinterface2.h>

namespace py = pybind11;
using namespace XBot::Cartesian::Utils;

PYBIND11_MODULE(pyest, m) {

    py::class_<ForceEstimation>(m, "ForceEstimation")
        .def(py::init<XBot::ModelInterface::ConstPtr, double>())
        .def("addLink", &ForceEstimation::add_link)
        .def("update", &ForceEstimation::update)
        .def("setIgnoredJoint", &ForceEstimation::setIgnoredJoint)
        ;

    py::class_<ForceEstimationMomentumBased, ForceEstimation>(m, "ForceEstimationMomentumBased")
        .def(py::init<XBot::ModelInterface::ConstPtr, double, double, double>(), py::arg("model"), py::arg("rate"), py::arg("svd_th"), py::arg("obs_bw"))
        ;
        
}

