#ifndef _CARTESIAN_MARKER_H_
#define _CARTESIAN_MARKER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>

namespace XBot { namespace Cartesian {

class CartesianMarker{
    
public:
    
    typedef std::shared_ptr<CartesianMarker> Ptr;
    
    CartesianMarker(const std::string& base_link, 
                    const std::string& distal_link,
                    const urdf::Model& robot_urdf,
                    std::string tf_prefix = "");
    
    ~CartesianMarker();

    /**
     * @brief clearMarker will remove the marker from the server
     * @param req
     * @param res
     * @return true
     */
    bool clearMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    /**
     * @brief spawnMarker will spawn the marker (if was cleared) in the actual pose of distal_link
     * @param req
     * @param res
     * @return true
     */
    bool spawnMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    
    void setBaseLink(std::string base_link);

    bool setGlobal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    bool setLocal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    /**
     * @brief setContinuous control mode
     * @param req
     * @param res
     * @return
     */
    bool setContinuous(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    /**
     * @brief setTrj control mode (enable waypoints)
     * @param req
     * @param res
     * @return
     */
    bool setTrj(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);


private:
    /**
     * @brief _nh
     */
    ros::NodeHandle _nh;

    /**
     * @brief _start_pose is the starting pose of the marker, taken from the current pose of the robot
     */
    KDL::Frame _start_pose;
    KDL::Frame _actual_pose;
    
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
    visualization_msgs::InteractiveMarker _int_marker;

    /**
     * @brief control
     */
    visualization_msgs::InteractiveMarkerControl _control;

    /**
     * @brief _control2
     */
    visualization_msgs::InteractiveMarkerControl _control2;

    /**
     * @brief _marker
     */
    visualization_msgs::Marker _marker;

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
    visualization_msgs::InteractiveMarkerControl  _menu_control;
    int _control_type;
    int _menu_entry_counter;
    int _is_continuous;
    int offset_menu_entry;

    tf::TransformListener _listener;
    tf::StampedTransform _transform;
    
    ros::Publisher _ref_pose_pub;

    ros::ServiceServer _clear_service;
    ros::ServiceServer _spawn_service;
//    ros::ServiceServer _global_service;
//    ros::ServiceServer _local_service;

    /**
     * @brief _waypoints contains all the waypoints BUT not the initial position!
     */
    std::vector<geometry_msgs::Pose> _waypoints;
    /**
     * @brief _T contains the times of each waypoint-trajectory
     */
    std::vector<double> _T;


    /**
     * @brief MakeMarker
     * @param distal_link
     * @param base_link
     * @param fixed
     * @param interaction_mode
     * @param show_6dof
     */
    void MakeMarker( const std::string& distal_link, const std::string& base_link,
                bool fixed, unsigned int interaction_mode, bool show_6dof);

    /**
     * @brief makeSTLControl return an Interactive Marker Control which uses an STL
     * @param msg an interactive marker msg
     * @return the interactive marker control
     */
    visualization_msgs::InteractiveMarkerControl& makeSTLControl(
            visualization_msgs::InteractiveMarker &msg );

    /**
     * @brief makeSTL uses an STL for an interactive marker
     * @param msg an interactive marker msg
     * @return  the interactive marker
     */
    visualization_msgs::Marker makeSTL( visualization_msgs::InteractiveMarker &msg );

    /**
     * @brief getRobotActualPose uses the tf to retrieve the actual pose of the robot
     * @return a KDL frame
     */
    KDL::Frame getRobotActualPose();

    KDL::Frame getPose(const std::string& base_link, const std::string& distal_link);

    void MarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void MakeMenu();

    void setControlGlobalLocal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void setContinuousCtrl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void wayPointCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void resetMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void resetAllWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void resetLastWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void sendWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    ros::Publisher _way_points_pub;
    void publishWP(const std::vector<geometry_msgs::Pose>& wps);
};

} }

#endif
