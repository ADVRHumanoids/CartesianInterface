#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <Eigen/Dense>

namespace py = pybind11;

Eigen::Vector3d get_t(const Eigen::Affine3d& T)
{
    return T.translation();
}

void set_t(Eigen::Affine3d& T, const Eigen::Vector3d& t)
{
    T.translation() = t;
}

Eigen::Vector4d get_q(const Eigen::Affine3d& T)
{
    return Eigen::Quaterniond(T.linear()).coeffs();
}

void set_q(Eigen::Affine3d& T, const Eigen::Vector4d& q)
{
    T.linear() = Eigen::Quaterniond(q).normalized().toRotationMatrix();
}

Eigen::Matrix3d get_lin(const Eigen::Affine3d& T)
{
    return T.linear();
}

void set_lin(Eigen::Affine3d& T, const Eigen::Matrix3d& R)
{
    T.linear() = R;
}

Eigen::Affine3d get_inverse(const Eigen::Affine3d& T)
{
    return T.inverse();
}

Eigen::Affine3d get_copy(const Eigen::Affine3d& T)
{
    return T;
}

Eigen::Affine3d construct(const Eigen::Vector3d& t = Eigen::Vector3d::Zero(), 
                          const Eigen::Vector4d& q = Eigen::Vector4d(0,0,0,1))
{
    Eigen::Affine3d T;
    T.setIdentity();
    T.linear() = Eigen::Quaterniond(q).normalized().toRotationMatrix();
    T.translation() = t;
    return T;
}

std::string repr(const Eigen::Affine3d& T)
{
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    
    std::stringstream ss;
    ss <<   "translation: " << T.translation().transpose().format(CleanFmt);
    ss << "\nrotation   : " << Eigen::Quaterniond(T.linear()).coeffs().transpose().format(CleanFmt);
    return ss.str();
}

PYBIND11_MODULE(affine3, m) {

    py::class_<Eigen::Affine3d>(m, "Affine3")
    .def(py::init(&construct), 
         py::arg("pos") = Eigen::Vector3d::Zero(), 
         py::arg("rot") = Eigen::Vector4d(0,0,0,1)
        )
    .def(py::init<Eigen::Affine3d>())
    .def("copy", get_copy)
    .def("__repr__", repr)
    .def("setIdentity", &Eigen::Affine3d::setIdentity)
    .def("inverse", get_inverse)
    .def(py::self * py::self)
    .def(py::self * Eigen::Vector3d())
    .def_property("quaternion", get_q, set_q)
    .def_property("linear", get_lin, set_lin)
    .def_property("translation", get_t, set_t);
    
}