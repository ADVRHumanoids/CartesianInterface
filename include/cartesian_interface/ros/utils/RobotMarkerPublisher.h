#ifndef CARTESIO_ROBOT_VIZ_H
#define CARTESIO_ROBOT_VIZ_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <xbot2_interface/xbotinterface2.h>


namespace XBot::Cartesian::Utils {

/**
 * @brief The RobotMarkerPublisher class is used to publish a marker array with all the meshes (collision & visual) of the robot
 * in a uniform color.
 */
class RobotMarkerPublisher
{

public:


    typedef Eigen::Vector4d color; // rgba

    /**
     * @brief RobotMarkerPublisher
     * @param model model of the robot to publish as marker
     * @param topic_name topic of the published marker
     * @param nh to retrieve the namespace of the topic
     * @param rgba color of the robot published
     */
    RobotMarkerPublisher(ModelInterface::ConstPtr model,
             std::string topic_name,
             rclcpp::Node::SharedPtr node = nullptr,
             std::optional<color> rgba = std::nullopt);

    RobotMarkerPublisher(ModelInterface::ConstPtr model,
             std::string topic_name,
             std::optional<color> rgba = std::nullopt);

    /**
     * @brief setRGBA
     * @param rgba [Red, Green, Blue, Alpha]
     */
    void setRGBA(const color& rgba);

    /**
     * @brief publishMarkers of the robot with a time
     * @param time
     * @param color_override_map if a link is in the map it will be published in the specified color
     */
    void publishMarkers(const rclcpp::Time& time, 
        std::string base_link = "world",
        std::string frame_id = "world",
        std::map<std::string, color> color_override_map = std::map<std::string, color>());

    /**
     * @brief getNode
     * @return
     */
    rclcpp::Node& getNode();

private:

    /**
     * @brief _reserved_color for the self collision
     */
    const color _reserved_color;

    XBot::ModelInterface::ConstPtr _model;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _collision_robot_pub;
    color _rgba;

    static Eigen::Affine3d toAffine3d(const urdf::Pose& p);

};

}


#endif // ROBOT_VIZ_H