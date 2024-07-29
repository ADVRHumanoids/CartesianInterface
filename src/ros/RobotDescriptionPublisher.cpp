#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("robot_description_publisher");

    auto urdf = node->declare_parameter<std::string>("robot_description");

    auto srdf = node->declare_parameter<std::string>("robot_description_semantic", "");

    auto urdf_pub = node->create_publisher<std_msgs::msg::String>(
        "robot_description",
        rclcpp::QoS(1).transient_local());

    std_msgs::msg::String string_msg;

    string_msg.data = urdf;

    urdf_pub->publish(string_msg);

    auto srdf_pub = urdf_pub;

    if(!srdf.empty())
    {
        srdf_pub = node->create_publisher<std_msgs::msg::String>(
            "robot_description_semantic",
            rclcpp::QoS(1).transient_local());

        string_msg.data = srdf;

        srdf_pub->publish(string_msg);
    }

    rclcpp::spin(node);
}
