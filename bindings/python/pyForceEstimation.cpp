#include <pyRosImpl.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <XBotInterface/ModelInterface.h>

namespace py = pybind11;
using namespace XBot::Cartesian::Utils;

PYBIND11_MODULE(pyest, m) {

    py::class_<ForceEstimation>(m, "ForceEstimation")
        .def(py::init<XBot::ModelInterface::ConstPtr, double>())
        .def("addLink", &ForceEstimation::add_link)
        .def("update", &ForceEstimation::update)
        ;
}


//        .def("sendForce", (void (ForcePublisher::*)(std::vector<std::string>, Eigen::VectorXd)), &ForcePublisher::send_force)
