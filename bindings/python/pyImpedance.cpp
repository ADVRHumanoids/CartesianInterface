#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <Eigen/Dense>

#include <cartesian_interface/problem/Interaction.h>

namespace py = pybind11;

using namespace XBot::Cartesian;

std::string impedance_repr(const Impedance& imp)
{
    std::stringstream ss;
    ss <<  "stiffness:\n" << imp.stiffness;
    ss << "\ndamping :\n" << imp.damping;
    ss << "\nmass    :\n" << imp.mass;
    return ss.str();
}

PYBIND11_MODULE(impedance, m) {

    py::class_<Impedance>(m, "Impedance")
    .def(py::init<const Eigen::Matrix6d&, const Eigen::Matrix6d&, const Eigen::Matrix6d&>(),
         py::arg("stiffness") = Eigen::Matrix6d::Zero(),
         py::arg("damping") = Eigen::Matrix6d::Zero(),
         py::arg("mass") = Eigen::Matrix6d::Identity())
        .def_readwrite("stiffness", &Impedance::stiffness)
        .def_readwrite("damping", &Impedance::damping)
        .def_readwrite("mass", &Impedance::mass)
        .def("__repr__", impedance_repr);
}
