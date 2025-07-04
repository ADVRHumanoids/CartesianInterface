#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <rclcpp/rclcpp.hpp>
#include <list>

bool init(std::string name, std::list<std::string> args)
{
    if(rclcpp::ok())
    {
        //ROS_ERROR("Ros node already initialized with name %s", 
        //          ros::this_node::getName().c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Ros node already initialized");
        
        return false;
    }
    
    std::vector<const char *> args_vec;
    for(auto& a : args)
    {
        args_vec.push_back(a.c_str());
    }
    
    int argc = args_vec.size();
    
    char ** argv = (char **)args_vec.data();
    
    name += "_cpp";
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(name);

    RCLCPP_INFO(node->get_logger(), 
                "Initialized roscpp under namespace %s with name %s", 
                    node->get_namespace(),
                    node->get_name()
            );
    
    return true;
}

bool shutdown()
{
    if(rclcpp::ok())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down ros node");

        rclcpp::shutdown();
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