#include <cartesian_interface/ros/utils/RobotMarkerPublisher.h>
#include <xbot2_interface/ros2/config_from_param.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xbot2_interface/collision.h>

class CollisionMarkerPublisherNode : public rclcpp::Node
{

public:

    CollisionMarkerPublisherNode(const std::string& name) : Node(name)
    {
    }

    void initialize()
    {
        auto cfg = XBot::ConfigOptionsFromParams(shared_from_this(), "", 5s);

        _model = XBot::ModelInterface::getModel(cfg);

        _marker_publisher = std::make_unique<XBot::Cartesian::Utils::RobotMarkerPublisher>(
            _model, 
            "collision_markers",
            shared_from_this(),
            Eigen::Vector4d(0, 0, 0, 0)
        );

        XBot::Collision::CollisionModel::Options opt;
        opt.assume_convex_meshes = declare_parameter("assume_convex_meshes", true);

        _collision_model = std::make_shared<XBot::Collision::CollisionModel>(_model, opt);

        _link_pairs = _collision_model->getCollisionPairs(true);

        _collision_color << 1, 0, 0, .8;

        _js_sub = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 
            rclcpp::QoS(1), 
            std::bind(&CollisionMarkerPublisherNode::on_js_recv, this, std::placeholders::_1)
        );

        
    }

private:



    void on_js_recv(sensor_msgs::msg::JointState::ConstSharedPtr js)
    {

        if(js->name.size() != js->position.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Received joint state with different name/position size");
            return;
        }

        XBot::JointNameMap qmap;

        for(size_t i=0; i<js->name.size(); ++i)
        {
            qmap[js->name[i]] = js->position[i];
        }

        _model->setJointPosition(qmap);
        _model->update();
        _collision_model->update();

        std::vector<int> pair_idx;

        _collision_model->checkCollision(pair_idx, true);
        
        std::map<std::string, Eigen::Vector4d> link_colors;
        for(auto idx : pair_idx)
        {
            link_colors[_link_pairs[idx].first] = _collision_color;
            link_colors[_link_pairs[idx].second] = _collision_color;

            RCLCPP_WARN(this->get_logger(), "Collision detected between links %s and %s", 
                        _link_pairs[idx].first.c_str(), _link_pairs[idx].second.c_str());
        }

        _marker_publisher->publishMarkers(get_clock()->now(), "world", "ci/world", link_colors);

        
    }

    XBot::ModelInterface::Ptr _model;
    XBot::Collision::CollisionModel::Ptr _collision_model;
    XBot::Collision::CollisionModel::LinkPairVector _link_pairs;
    std::unique_ptr<XBot::Cartesian::Utils::RobotMarkerPublisher> _marker_publisher;
    Eigen::Vector4d _collision_color;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _js_sub;


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CollisionMarkerPublisherNode>("collision_marker_publisher");

    node->initialize();

    RCLCPP_INFO(node->get_logger(), "Node initialized");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
