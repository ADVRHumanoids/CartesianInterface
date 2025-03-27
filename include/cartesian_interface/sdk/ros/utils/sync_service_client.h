#ifndef CI_SYNC_SERVICE_CLIENT_H
#define CI_SYNC_SERVICE_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

template<typename SrvType>
class SyncServiceClient
{

public:

    typedef std::shared_ptr<SyncServiceClient> SharedPtr;

    typedef typename SrvType::Request Request;
    typedef typename SrvType::Response Response;

    SyncServiceClient(rclcpp::Node::SharedPtr node,
                      std::string service_name, 
                      std::chrono::seconds timeout = std::chrono::seconds(1)):
        _node(node)
    {
        _exe = rclcpp::executors::SingleThreadedExecutor::make_shared();

        _cb_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, 
                                                false);

        _exe->add_callback_group(_cb_group, _node->get_node_base_interface());

        _cli = node->create_client<SrvType>(service_name, 
                                            rmw_qos_profile_services_default, 
                                            _cb_group);

        while(!_cli->wait_for_service(timeout))
        {
            RCLCPP_INFO_STREAM(_node->get_logger(),
                               "waiting for service '" << _cli->get_service_name() << "'");
        }
    }

    const char * get_service_name() const
    {
        return _cli->get_service_name();
    }

    rclcpp::Client<SrvType>::SharedPtr get_client() const
    {
        return _cli;
    }

    std::shared_ptr<Response> call(std::shared_ptr<Request> req, 
        std::chrono::seconds timeout = std::chrono::seconds(1))
    {
        auto fut = _cli->async_send_request(req);

        if(_exe->spin_until_future_complete(fut, timeout) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return fut.get();
        }
        else
        {
            RCLCPP_ERROR_STREAM(_node->get_logger(), "service call failed ('" << 
                _cli->get_service_name() << "')");
            return nullptr;
        }
    }
    
private:

    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _cb_group;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr _exe;
    rclcpp::Client<SrvType>::SharedPtr _cli;

};

#endif // SYNC_SERVICE_CLIENT_H