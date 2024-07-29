#ifndef _CARTESIAN_MARKER_H_
#define _CARTESIAN_MARKER_H_

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>

#include <cartesian_interface/action/reach_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cartesian_interface/problem/Cartesian.h>


namespace XBot { namespace Cartesian {

using EmptySrv = std_srvs::srv::Empty;
using visualization_msgs::msg::InteractiveMarkerFeedback;
using geometry_msgs::msg::PoseArray;

class CartesianMarker
{
    
public:
    
    typedef std::shared_ptr<CartesianMarker> Ptr;
    
    /**
     * @brief CartesianMarker
     * @param base_link
     * @param distal_link
     * @param robot_urdf
     * @param control_type is the type of control:
     *  visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D
     *  visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D
     *  visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D
     * @param tf_prefix
     * @param use_mesh, if false sphere is used
     */
    CartesianMarker(const std::string& task_name,
                    const urdf::Model& robot_urdf,
                    const unsigned int control_type,
                    std::string tf_prefix = "",
                    const bool use_mesh = true);

    CartesianMarker(const std::string& task_name,
                    const urdf::Model& robot_urdf,
                    const unsigned int control_type,
                    rclcpp::Node::SharedPtr n,
                    std::string tf_prefix = "",
                    const bool use_mesh = true);
    
    ~CartesianMarker();

    /**
     * @brief clearMarker will remove the marker from the server
     * @param req
     * @param res
     * @return true
     */
    bool clearMarker(EmptySrv::Request::ConstSharedPtr req,
                     EmptySrv::Response::SharedPtr res);

    /**
     * @brief spawnMarker will spawn the marker (if was cleared) in the actual pose of distal_link
     * @param req
     * @param res
     * @return true
     */
    bool spawnMarker(EmptySrv::Request::ConstSharedPtr req,
                     EmptySrv::Response::SharedPtr res);
    
    void setBaseLink(std::string base_link);

    bool setGlobal(EmptySrv::Request::ConstSharedPtr req,
                   EmptySrv::Response::SharedPtr res);

    bool setLocal(EmptySrv::Request::ConstSharedPtr req,
                  EmptySrv::Response::SharedPtr res);

    /**
     * @brief setContinuous control mode
     * @param req
     * @param res
     * @return
     */
    bool setContinuous(EmptySrv::Request::ConstSharedPtr req,
                       EmptySrv::Response::SharedPtr res);

    /**
     * @brief setTrj control mode (enable waypoints)
     * @param req
     * @param res
     * @return
     */
    bool setTrj(EmptySrv::Request::ConstSharedPtr req,
                EmptySrv::Response::SharedPtr res);


private:

    CartesianTask::Ptr _task;

    /**
     * @brief _nh
     */
    rclcpp::Node::SharedPtr _node;

    /**
     * @brief _start_pose is the starting pose of the marker, taken from the current pose of the robot
     */
    Eigen::Affine3d _start_pose;
    Eigen::Affine3d _actual_pose;
    
    std::string _tf_prefix;
    
    /**
     * @brief _base_link used by the marker
     */
    std::string _base_link;
    /**
     * @brief _distal_link used by the marker
     */
    std::string _distal_link;
    /**
     * @brief _urdf model description of the robot
     */
    urdf::Model _urdf;
    /**
     * @brief _server interactive marker server used by the marker
     */
    interactive_markers::InteractiveMarkerServer _server;
    /**
     * @brief _int_marker this is the actual interactive marker
     */
    visualization_msgs::msg::InteractiveMarker _int_marker;

    /**
     * @brief control
     */
    visualization_msgs::msg::InteractiveMarkerControl _control;

    /**
     * @brief _control2
     */
    visualization_msgs::msg::InteractiveMarkerControl _control2;

    /**
     * @brief _marker
     */
    visualization_msgs::msg::Marker _marker;

    interactive_markers::MenuHandler _menu_handler;
    interactive_markers::MenuHandler::EntryHandle _reset_marker_entry;
    interactive_markers::MenuHandler::EntryHandle _way_point_entry;
    interactive_markers::MenuHandler::EntryHandle _T_entry;
    interactive_markers::MenuHandler::EntryHandle _T_last;
    interactive_markers::MenuHandler::EntryHandle _reset_last_way_point_entry;
    interactive_markers::MenuHandler::EntryHandle _reset_all_way_points_entry;
    interactive_markers::MenuHandler::EntryHandle _send_way_points_entry;
    interactive_markers::MenuHandler::EntryHandle _global_control_entry;
    interactive_markers::MenuHandler::EntryHandle _continuous_control_entry;
    interactive_markers::MenuHandler::EntryHandle _properties_entry;
    interactive_markers::MenuHandler::EntryHandle _task_is_active_entry;
    interactive_markers::MenuHandler::EntryHandle _position_feedback_is_active_entry;
    interactive_markers::MenuHandler::EntryHandle _base_link_entry;
    std::vector<interactive_markers::MenuHandler::EntryHandle> _link_entries;
    visualization_msgs::msg::InteractiveMarkerControl  _menu_control;
    int _control_type;
    int _menu_entry_counter;
    int _is_continuous;
    int offset_menu_entry;
    int _task_active;
    int _position_feedback_active;

    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> _listener;
    
    rclcpp::ServiceBase::SharedPtr _clear_service;
    rclcpp::ServiceBase::SharedPtr _spawn_service;

    EmptySrv::Request::SharedPtr _req;
    EmptySrv::Response::SharedPtr _res;

    bool _use_mesh;

    /**
     * @brief _waypoints contains all the waypoints BUT not the initial position!
     */
    std::vector<geometry_msgs::msg::Pose> _waypoints;
    /**
     * @brief _T contains the times of each waypoint-trajectory
     */
    std::vector<float> _T;

    std::vector<urdf::LinkSharedPtr> _links;
    int _base_link_entry_active;
    std::map<std::string, int> _map_link_entry;

    /**
     * @brief MakeMarker
     * @param distal_link
     * @param base_link
     * @param fixed
     * @param interaction_mode
     * @param show
     */
    void MakeMarker( const std::string& distal_link, const std::string& base_link,
                bool fixed, unsigned int interaction_mode, bool show);

    /**
     * @brief makeSTLControl return an Interactive Marker Control which uses an STL
     * @param msg an interactive marker msg
     * @return the interactive marker control
     */
    visualization_msgs::msg::InteractiveMarkerControl& makeSTLControl(
        visualization_msgs::msg::InteractiveMarker &msg );

    /**
     * @brief makeSTL uses an STL for an interactive marker
     * @param msg an interactive marker msg
     * @return  the interactive marker
     */
    visualization_msgs::msg::Marker makeSTL( visualization_msgs::msg::InteractiveMarker &msg );

    visualization_msgs::msg::Marker makeSphere( visualization_msgs::msg::InteractiveMarker &msg );

    /**
     * @brief getRobotActualPose uses the tf to retrieve the actual pose of the robot
     * @return a KDL frame
     */
    Eigen::Affine3d getRobotActualPose();

    Eigen::Affine3d getPose(const std::string& base_link, const std::string& distal_link);

    void MarkerFeedback(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void MakeMenu();

    void setControlGlobalLocal(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void setContinuousCtrl(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void wayPointCallBack(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void resetMarker(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void resetAllWayPoints(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void resetLastWayPoints(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void sendWayPoints(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void _activateTask(const bool is_active);
    void activateTask(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void _activatePositionFeedBack(const bool is_active);
    void activatePositionFeedBack(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    void changeBaseLink(InteractiveMarkerFeedback::ConstSharedPtr feedback);

    rclcpp::Publisher<PoseArray>::SharedPtr _way_points_pub;
    void publishWP(const std::vector<geometry_msgs::msg::Pose>& wps);

    void createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                        const unsigned int interaction_mode);
};

} }

#endif
