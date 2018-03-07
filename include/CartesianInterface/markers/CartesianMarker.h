#ifndef _CARTESIAN_MARKER_H_
#define _CARTESIAN_MARKER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_listener.h>

namespace XBot { namespace Cartesian {

class CartesianMarker{
    
public:
    
    typedef std::shared_ptr<CartesianMarker> Ptr;
    
    CartesianMarker(const std::string& base_link, 
                    const std::string& distal_link,
                    const urdf::Model& robot_urdf,
                    std::string tf_prefix = "");
    
    ~CartesianMarker();

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

    tf::TransformListener _listener;
    tf::StampedTransform _transform;
    
    ros::Publisher _ref_pose_pub;

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
};

} }

#endif
