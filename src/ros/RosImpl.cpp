#include <cartesian_interface/ros/RosClient.h>
#include <cartesian_interface_ros/srv/reset_world.hpp>
#include <cartesian_interface_ros/srv/load_controller.hpp>
#include <cartesian_interface_ros/srv/get_task_list.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
//#include <tf_conversions/tf_eigen.h>
//#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/srv/trigger.hpp>

#include "fmt/format.h"

#include "ros/client_api/TaskRos.h"
#include "ros/client_api/CartesianRos.h"
#include "ros/client_api/InteractionRos.h"

#include <xbot2_interface/logger.h>

#define THROW_NOT_IMPL throw std::runtime_error("Not implemented function " + std::string(__func__));

using namespace XBot::Cartesian;

using namespace std::chrono_literals;

namespace
{

ProblemDescription construct_problem(rclcpp::Node::SharedPtr node)
{

    auto get_task_list_srv = node->create_client<cartesian_interface_ros::srv::GetTaskList>("get_task_list");

    // int attempts = 100;
    // while(attempts-- && !get_task_list_srv.exists())
    // {
    //     if(attempts % 10 == 0)
    //     {
    //         fmt::print("Trying to contact server for service '{}' ... \n",
    //                    get_task_list_srv->get_service_name());
    //     }

    //     usleep(0.1 * 1e6);
    // }

    if(!get_task_list_srv->wait_for_service(1s))
    {
        if (!rclcpp::ok()) {
            throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
        }
        throw std::runtime_error(fmt::format("Service '{}' not available",
                                             get_task_list_srv->get_service_name()));
    }

    auto srv_list_req = std::make_shared<cartesian_interface_ros::srv::GetTaskList::Request>();
    auto srv_list_res = get_task_list_srv->async_send_request(srv_list_req);
    
    if (rclcpp::spin_until_future_complete(node, srv_list_res) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        throw std::runtime_error(fmt::format("Failed to call service '{}'",
            get_task_list_srv->get_service_name()));
    } 

    AggregatedTask tasks;
    auto srv_list = srv_list_res.get();
    for(int i = 0; i < srv_list->names.size(); i++)
    {
        std::string name = srv_list->names[i];

        auto get_task_info_srv = node->create_client<cartesian_interface_ros::srv::GetTaskInfo>(
            name + "/get_task_properties");

        if(!get_task_info_srv->wait_for_service(1s))
        {
            if (!rclcpp::ok()) {
                throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
            }
            throw std::runtime_error(fmt::format("Service '{}' not available",
                get_task_info_srv->get_service_name()));
        }

        auto srv_info_req = std::make_shared<cartesian_interface_ros::srv::GetTaskInfo::Request>();
        auto srv_info_res = get_task_info_srv->async_send_request(srv_info_req);

        if (rclcpp::spin_until_future_complete(node, srv_info_res) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            throw std::runtime_error(fmt::format("Failed to call service '{}'",
                get_task_info_srv->get_service_name()));
        } 

        auto srv_info = srv_info_res.get();

        auto t = ClientApi::TaskRos::MakeInstance(name,
                                                  srv_info->type,
                                                  srv_info->lib_name,
                                                  node);

        tasks.push_back(t);

    }

    ProblemDescription ik_pb(tasks);

    int attempts = 100;
    //TODO how to use callback group, when it must be given to the subscribers?
    //is it so bad to use the generic rclcpp::spin here?
    
    //auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //rclcpp::executors::SingleThreadedExecutor exec;
    //exec.add_callback_group(cbg, node->get_node_base_interface()); 

    while(--attempts && !ik_pb.validate())
    {
        if(attempts % 10 == 0) fmt::print("Waiting for all tasks to become valid... \n");

        // auto queue = static_cast<ros::CallbackQueue*>(nh.getCallbackQueue());
        // queue->callAvailable();

        //exec.spin_all(1s); //or spin_all(); or spin_once???
        rclcpp::spin_some(node);
        usleep(0.1 * 1e6);
    }

    if(attempts == 0)
    {
        ik_pb.validate(true);
    }

    return ik_pb;

}

}

std::ostream& XBot::Cartesian::operator<<(std::ostream& os, const RosClient& r)
{
    os << "CartesianInterfaceRos running inside ROS node " << r._ns << "\n";
    auto tasklist = r.getTaskList();
    os << "Defined tasks: \n";
    for(auto t : tasklist)
    {
        os << " - ";
        os << XBot::bold_on << t << XBot::bold_off << "\n";
    }
    
    return os;
    
}

RosInitializer::RosInitializer(std::string ns)
{

    bool first_init = false;
    if(!rclcpp::ok())
    {
        std::string ns_arg = "__ns:=";
        ns_arg += "";
        std::vector<const char *> args {"", ns_arg.c_str()};

        int argc = args.size();

        rclcpp::init(argc, (char **)args.data());
        
        first_init = true;
    }

    //ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    _node = rclcpp::Node::make_shared("cartesio_ros", ns);
    _ros2_cbg = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    if (first_init) {
        RCLCPP_WARN(_node->get_logger(),
            "Initializing rclcpp under namespace '%s' with name '%s'",
            _node->get_namespace(),
            _node->get_name()
        );
    } else {
        RCLCPP_WARN(_node->get_logger(),
            "Recreating the node under namespace '%s' with name '%s'. Ported from ROS1, ask Arturo if this is correct",
            _node->get_namespace(),
            _node->get_name()
        );
    }

    // TODO correct porting?
    // _nh.reset(new ros::NodeHandle(ns));
    // _nh->setCallbackQueue(&_queue);
    _exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _exec->add_callback_group(_ros2_cbg, _node->get_node_base_interface()); 

}

rclcpp::Node::SharedPtr RosInitializer::node()
{
    return _node;
}

void RosInitializer::callAvailable()
{
    // TODO correcT?
    //_queue.callAvailable();
    _exec->spin_some();
}


RosClient::RosClient(std::string ns):
    _ns(ns),
    RosInitializer(ns),
    CartesianInterfaceImpl(::construct_problem(node()))
{
    _load_ctrl_srv = node()->create_client<cartesian_interface_ros::srv::LoadController>("load_controller");

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(node()->get_clock());
    _listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

}

void RosClient::set_async_mode(bool async)
{
    for(auto t : getTaskList())
    {
        if(auto tros = std::dynamic_pointer_cast<ClientApi::TaskRos>(getTask(t)))
        {
            tros->setAsyncMode(async);
        }
    }
}

bool RosClient::setVelocityReference(const std::string& end_effector,
                                     const Eigen::Vector6d& base_vel_ref,
                                     const std::string& base_frame)
{
    auto cart_ros = std::dynamic_pointer_cast<ClientApi::CartesianRos>(getTask(end_effector));

    if(!cart_ros) return false;

    return cart_ros->setVelocityReference(base_vel_ref, base_frame);
}

bool RosClient::setWayPoints(const std::string& end_effector,
                             const Trajectory::WayPointVector& way_points,
                             bool incremental)
{
    auto cart_ros = std::dynamic_pointer_cast<ClientApi::CartesianRos>(getTask(end_effector));

    if(!cart_ros) return false;

    return cart_ros->setWayPoints(way_points, incremental);
}

bool XBot::Cartesian::RosClient::getPoseFromTf(const std::string& source_frame,
                                               const std::string& target_frame,
                                               Eigen::Affine3d& t_T_s)
{

    geometry_msgs::msg::TransformStamped T;

    // if(!_tf_buffer->waitForTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)))
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Wait for transform timed out");
    //     return false;
    // }

    try {

        T = _tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), "Could not transform %s to %s: %s",
            source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }

    //tf::transformTFToEigen(T, t_T_s);
    t_T_s.translation().x() = T.transform.translation.x;
    t_T_s.translation().y() = T.transform.translation.y;
    t_T_s.translation().z() = T.transform.translation.z;
    
    t_T_s.linear() = Eigen::Quaterniond(T.transform.rotation.w,
                                        T.transform.rotation.x,
                                        T.transform.rotation.y,
                                        T.transform.rotation.z).toRotationMatrix();

    return true;
}

bool RosClient::update(double time, double period)
{
    callAvailable();
    return true;
}

void RosClient::loadController(const std::string& controller_name,
                             const std::string& problem_description_name,
                             const std::string& problem_description_string,
                             const bool force_reload)
{
    auto load_controller_req = std::make_shared<cartesian_interface_ros::srv::LoadController::Request>();
    load_controller_req->controller_name = controller_name;
    load_controller_req->force_reload = force_reload;
    load_controller_req->problem_description_name = problem_description_name;
    load_controller_req->problem_description_string = problem_description_string;

    while (!_load_ctrl_srv->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(node()->get_logger(), "service not available, waiting again...");
    }

    auto load_controller_res = _load_ctrl_srv->async_send_request(load_controller_req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node(), load_controller_res) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        throw std::runtime_error(fmt::format("Unable to connect to '{}'",
            _load_ctrl_srv->get_service_name()));
    } 

    if(!load_controller_res.get()->success)
    {
        throw std::runtime_error(fmt::format("{}  responded with an error:\n\t{}",
            _load_ctrl_srv->get_service_name(),
            load_controller_res.get()->message));
    }

    RCLCPP_INFO(node()->get_logger(), "%s", load_controller_res.get()->message.c_str());

    std::string ns = node()->get_namespace();
    this->~RosClient();
    new(this) RosClient(ns);


}

bool RosClient::waitReachCompleted(const std::string & ee_name, double timeout_sec)
{
    auto cart_ros = std::dynamic_pointer_cast<ClientApi::CartesianRos>(getTask(ee_name));

    if(!cart_ros) return false;

    return cart_ros->waitReachCompleted(timeout_sec);
}

bool RosClient::setStiffnessTransition(const std::string& end_effector,
                      const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points)
{
    auto interaction_ros = std::dynamic_pointer_cast<ClientApi::InteractionRos>(getTask(end_effector));

    if(!interaction_ros) return false;

    return interaction_ros->setStiffnessTransition(way_points);
}

bool RosClient::waitStiffnessTransitionCompleted(const std::string& ee_name, double timeout_sec)
{
    auto interaction_ros = std::dynamic_pointer_cast<ClientApi::InteractionRos>(getTask(ee_name));

    if(!interaction_ros) return false;

    return interaction_ros->waitTransitionCompleted(timeout_sec);
}

bool RosClient::abortStiffnessTransition(const std::string& end_effector)
{
	auto interaction_ros = std::dynamic_pointer_cast<ClientApi::InteractionRos>(getTask(end_effector));

    if(!interaction_ros) return false;
	
	interaction_ros->abortStiffnessTransition();
    return true;
}

namespace
{
    bool call_reset_world_service(rclcpp::Node::SharedPtr node,
                                  const Eigen::Affine3d& w_T_new_world,
                                  const std::string& ee_name)
    {


        auto client = node->create_client<cartesian_interface_ros::srv::ResetWorld>("reset_world");
        if(!client->wait_for_service(3s))
        {
            throw std::runtime_error("unable to reset world, service unavailable");
        }

        auto reset_world_req = std::make_shared<cartesian_interface_ros::srv::ResetWorld::Request>();
        //tf::poseEigenToMsg(w_T_new_world, srv.request.new_world);
        reset_world_req->new_world.position.x = w_T_new_world.translation().x();
        reset_world_req->new_world.position.y = w_T_new_world.translation().y();
        reset_world_req->new_world.position.z = w_T_new_world.translation().z();
        Eigen::Quaterniond q(w_T_new_world.linear());
        reset_world_req->new_world.orientation.x = q.x();
        reset_world_req->new_world.orientation.y = q.y();
        reset_world_req->new_world.orientation.z = q.z();
        reset_world_req->new_world.orientation.w = q.w();

        reset_world_req->from_link = ee_name;

        auto reset_world_res = client->async_send_request(reset_world_req);
        if (rclcpp::spin_until_future_complete(node, reset_world_res) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            throw std::runtime_error("unable to reset world, service call failed");
        } 

        RCLCPP_INFO(node->get_logger(), "%s", reset_world_res.get()->message.c_str());

        if(!reset_world_res.get()->success)
        {
            throw std::runtime_error("unable to reset world, service responded with an error");
        }

        return true;
    }
}

bool RosClient::resetWorld(const Eigen::Affine3d& w_T_new_world)
{
    return ::call_reset_world_service(node(), w_T_new_world, "");
}

bool XBot::Cartesian::RosClient::resetWorld(const std::string& ee_name)
{
    return ::call_reset_world_service(node(), Eigen::Affine3d::Identity(), ee_name);
}




