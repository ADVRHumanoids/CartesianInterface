#include <cartesian_interface/ros/RosControlTopicApi.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <XBotInterface/RtLog.hpp>

using namespace XBot::Cartesian;

RosControlTopicAPI::RosControlTopicAPI(ros::NodeHandle& n, const std::string& hardware_interface, const double dt, ModelInterface::Ptr& model):
    _n(n),
    _model(model)
{
    Logger::info("Requested hardware_interface: %s \n", hardware_interface.c_str());

    ros::ServiceClient controller_manager_client = _n.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");

    controller_manager_msgs::ListControllers srv;

    if(!controller_manager_client.waitForExistence(ros::Duration(5)))
    {
        throw std::runtime_error("/controller_manager/list_controllers service does not exists!");
    }

    if(controller_manager_client.call(srv))
    {
        Logger::info("Available controllers:\n");
        for(const auto& controller : srv.response.controller)
        {
            Logger::info("name: %s\n", controller.name.c_str());
            Logger::info("  type: %s\n", controller.type.c_str());

            for(const auto& res : controller.claimed_resources)
            {
                Logger::info("  hardware interface: %s\n", res.hardware_interface.c_str());
                for(auto r : res.resources)
                    Logger::info("  - %s\n", r.c_str());
                if(res.hardware_interface == hardware_interface)
                {
                    _hw_interface_publishers.push_back(hw_interface_publisher(controller.name, res.resources, dt, _n));
                    Logger::info("Added this hardware interface publisher!\n");
                }
            }
        }

        if(_hw_interface_publishers.empty())
            throw std::runtime_error("There are no controllers with the requested hardware interface.");
    }
    else
    {
        throw std::runtime_error("Controller manager service returned false!");
    }
}

void RosControlTopicAPI::setReference(const Eigen::VectorXd& u)
{
    for(auto &hw : _hw_interface_publishers)
    {
        for(const auto& joint : hw.get_joints())
        {
            hw.set_joint_position(joint, u[_model->getDofIndex(joint)]);
        }
    }

    for(auto &hw : _hw_interface_publishers)
        hw.publish();
}

bool RosControlTopicAPI::startStoppedControllers()
{
    ros::ServiceClient controller_manager_client = _n.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");

    controller_manager_msgs::ListControllers srv;

    if(!controller_manager_client.waitForExistence(ros::Duration(5)))
    {
        Logger::error("/controller_manager/list_controllers service does not exists!");
        return false;
    }

    if(controller_manager_client.call(srv))
    {
        ros::ServiceClient controller_manager_client2 = _n.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

        controller_manager_msgs::SwitchController srv2;

        if(!controller_manager_client2.waitForExistence(ros::Duration(5)))
        {
            Logger::error("/controller_manager/switch_controller service does not exists!");
            return false;
        }

        for(const auto& controller : srv.response.controller)
        {
            for(const auto& hw : _hw_interface_publishers)
            {
                if(hw.get_interface_name() == controller.name)
                {
                    if(controller.state == "stopped")
                    {
                        srv2.request.start_controllers.push_back(controller.name);
                        Logger::info("Controller %s is stopped, will be started\n", controller.name.c_str());
                    }
                }
            }
            srv2.request.strictness = 2;
        }

        if(srv2.request.start_controllers.size() > 0 )
        {
            if(!controller_manager_client2.call(srv2))
            {
                Logger::error("Switch controller service returned false!");
                return false;
            }
        }
        return true;

    }
    else
    {
        Logger::error("Controller manager service returned false!");
        return false;
    }
}
