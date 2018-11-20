#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ros/ros.h>
#include <list>

void init(std::string name, std::list<std::string> args)
{
    std::vector<const char *> args_vec;
    for(auto a : args)
    {
        args_vec.push_back(a.c_str());
    }
    
    int argc = args_vec.size();
    
    char ** argv = (char **)args_vec.data();
    
    name += "_cpp";
    
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
}

namespace py = pybind11;

PYBIND11_MODULE(roscpp_utils, m) {
    
    m.def("init", init);
    
}