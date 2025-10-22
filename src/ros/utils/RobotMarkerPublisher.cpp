#include <cartesian_interface/ros/utils/RobotMarkerPublisher.h>
#include <memory>

using namespace XBot::Cartesian::Utils;

RobotMarkerPublisher::RobotMarkerPublisher(ModelInterface::ConstPtr model,
                   std::string topic_name,
                   rclcpp::Node::SharedPtr node,
                   std::optional<RobotMarkerPublisher::color> rgba):
    _model(model),
    _node(node ? node : rclcpp::Node::make_shared("RobotMarkerPublisher_node")),
    _reserved_color(1, 0, 0, 1)
{
    _collision_robot_pub = _node->create_publisher<visualization_msgs::msg::MarkerArray>( topic_name, 1);

    if(!rgba)
    {
        // green by default
        rgba = Eigen::Vector4d(0, 1, 0, 1);
    }

    if(rgba != _reserved_color)
    {
        _rgba = *rgba;
    }
    else
    {
        throw std::invalid_argument("invalid color");
    }
}

RobotMarkerPublisher::RobotMarkerPublisher(ModelInterface::ConstPtr model,
                                              std::string topic_name,
                                              std::optional<color> rgba):
    RobotMarkerPublisher(model, topic_name, nullptr, rgba)
{

}

void RobotMarkerPublisher::setRGBA(const color& rgba)
{
    _rgba = rgba;
}

void RobotMarkerPublisher::publishMarkers(const rclcpp::Time & time, 
        std::string base_link,
        std::string frame_id,
        std::map<std::string, color> color_override_map)
{

    visualization_msgs::msg::MarkerArray markers;

    rclcpp::Time t = time;

    std::vector<urdf::LinkSharedPtr> links;
    _model->getUrdf()->getLinks(links);

    int id = 0;

    auto add_marker = [&]
        (urdf::LinkSharedPtr link, urdf::Pose origin, urdf::GeometrySharedPtr geometry, std::string ns)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = frame_id;
        marker.header.stamp = t;
        marker.ns = ns;
        marker.id = id;

        marker.action = visualization_msgs::msg::Marker::ADD;

        Eigen::Affine3d pose; _model->getPose(link->name, base_link, pose);
        pose = pose*toAffine3d(origin);

        marker.pose.position.x = pose.translation()[0];
        marker.pose.position.y = pose.translation()[1];
        marker.pose.position.z = pose.translation()[2];
        Eigen::Quaterniond q(pose.linear());
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        auto it = color_override_map.find(link->name);

        if(it != color_override_map.end())
        {
            marker.color.a = it->second[3];
            marker.color.r = it->second[0];
            marker.color.g = it->second[1];
            marker.color.b = it->second[2];
        }
        else
        {
            marker.color.a = _rgba[3];
            marker.color.r = _rgba[0];
            marker.color.g = _rgba[1];
            marker.color.b = _rgba[2];
        }

        if(geometry->type == urdf::Geometry::BOX)
        {
            marker.type = visualization_msgs::msg::Marker::CUBE;

            auto mesh =
                std::static_pointer_cast<urdf::Box>(geometry);

            marker.scale.x = mesh->dim.x;
            marker.scale.y = mesh->dim.y;
            marker.scale.z = mesh->dim.z;
        }
        else if(geometry->type == urdf::Geometry::CYLINDER)
        {
            marker.type = visualization_msgs::msg::Marker::CYLINDER;

            auto mesh =
                std::static_pointer_cast<urdf::Cylinder>(geometry);

            marker.scale.x = marker.scale.y = 2.*mesh->radius;
            marker.scale.z = mesh->length;
        }
        else if(geometry->type == urdf::Geometry::SPHERE)
        {
            marker.type = visualization_msgs::msg::Marker::SPHERE;

            auto mesh =
                std::static_pointer_cast<urdf::Sphere>(geometry);

            marker.scale.x = marker.scale.y = marker.scale.z = 2.*mesh->radius;
        }
        else if(geometry->type == urdf::Geometry::MESH)
        {
            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;


            auto mesh =
                std::static_pointer_cast<urdf::Mesh>(geometry);

            marker.mesh_resource = mesh->filename;
            marker.scale.x = mesh->scale.x;
            marker.scale.y = mesh->scale.y;
            marker.scale.z = mesh->scale.z;
        }
        markers.markers.push_back(marker);
        id++;
    };

    for(auto link : links)
    {

        for(auto coll : link->collision_array)
        {
            add_marker(link, coll->origin, coll->geometry, "collision");
        }

        for(auto vis : link->visual_array)
        {
            add_marker(link, vis->origin, vis->geometry, "visual");
        }

    }

    _collision_robot_pub->publish(markers);
}

rclcpp::Node &RobotMarkerPublisher::getNode()
{
    return *_node;
}

Eigen::Affine3d RobotMarkerPublisher::toAffine3d(const urdf::Pose & p)
{
    Eigen::Affine3d T;

    T.translation()[0] = p.position.x;
    T.translation()[1] = p.position.y;
    T.translation()[2] = p.position.z;

    T.linear() = Eigen::Matrix3d(Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z));
    return T;
}