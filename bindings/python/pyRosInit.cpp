#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ros/ros.h>
#include <list>

bool init(std::string name, std::list<std::string> args)
{
    if(ros::ok())
    {
        ROS_ERROR("Ros node already initialized with name %s", 
                  ros::this_node::getName().c_str());
        return false;
    }
    
    std::vector<const char *> args_vec;
    for(auto a : args)
    {
        args_vec.push_back(a.c_str());
    }
    
    int argc = args_vec.size();
    
    char ** argv = (char **)args_vec.data();
    
    name += "_cpp";
    
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
    
    ROS_INFO("Initialized roscpp under namespace %s with name %s", 
             ros::this_node::getNamespace().c_str(),
             ros::this_node::getName().c_str()
            );
    
    return true;
}

bool shutdown()
{
    if(ros::ok())
    {
        ROS_INFO("Shutting down ros node");
        ros::shutdown();
        return true;
    }
    
    std::cerr << "Ros node not running" << std::endl;
    return false;
}


namespace py = pybind11;

PYBIND11_MODULE(roscpp_utils, m) {
    
    m.def("init", init);
    m.def("shutdown", shutdown);
    
}