#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <estimation_utils/payload/force_estimation.h>
#include <XBotInterface/ModelInterface.h>

namespace py = pybind11;
using namespace estimation_utils;

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

